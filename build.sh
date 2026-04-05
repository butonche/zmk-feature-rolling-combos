#!/usr/bin/env bash
set -euo pipefail

IMAGE="docker.io/zmkfirmware/zmk-dev-x86_64:4.1"
MODULE_DIR="$(cd "$(dirname "$0")" && pwd)"
TESTS_DIR="$MODULE_DIR/tests/rolling-combos"

docker run --rm \
  -v "$MODULE_DIR":/module:ro \
  -w /workspace \
  "$IMAGE" \
  bash -c '
set -euo pipefail

echo "=== Cloning ZMK ==="
git clone --depth 1 --quiet https://github.com/zmkfirmware/zmk.git /workspace/zmk
cd /workspace/zmk
west init -l app 2>&1 | tail -1
echo "=== Running west update (this takes a while) ==="
west update --fetch-opt=--filter=tree:0 > /dev/null 2>&1
echo "=== West update complete ==="

export ZMK_EXTRA_MODULES=/module

passed=0
failed=0
errors=""

for test_dir in /module/tests/rolling-combos/*/; do
    test_name="$(basename "$test_dir")"

    # Skip non-test entries (e.g. shared dtsi files)
    if [ ! -f "$test_dir/native_sim.keymap" ]; then
        continue
    fi

    echo ""
    echo "=== Building test: $test_name ==="
    if ! west build -p always -b native_sim//zmk_test_mock -d "/workspace/build/$test_name" app -- \
        -DZMK_CONFIG="$test_dir" \
        -DCONFIG_ASSERT=y \
        -DZMK_EXTRA_MODULES=/module 2>&1; then
        echo "FAIL (build error): $test_name"
        failed=$((failed + 1))
        errors="$errors  BUILD: $test_name\n"
        continue
    fi

    echo "--- Running test: $test_name ---"
    exe="/workspace/build/$test_name/zephyr/zmk.exe"
    if [ ! -f "$exe" ]; then
        exe="/workspace/build/$test_name/zephyr/zephyr.exe"
    fi
    output=$("$exe" 2>&1 || true)
    actual=$(echo "$output" | sed -n "$(cat "$test_dir/events.patterns")")

    if echo "$actual" | diff - "$test_dir/keycode_events.snapshot" > /dev/null 2>&1; then
        echo "PASS: $test_name"
        passed=$((passed + 1))
    else
        echo "FAIL: $test_name"
        echo "  Expected:"
        sed "s/^/    /" "$test_dir/keycode_events.snapshot"
        echo "  Actual:"
        echo "$actual" | sed "s/^/    /"
        echo "  Diff:"
        echo "$actual" | diff - "$test_dir/keycode_events.snapshot" | sed "s/^/    /" || true
        failed=$((failed + 1))
        errors="$errors  SNAPSHOT: $test_name\n"
    fi
done

echo ""
echo "=============================="
echo "Results: $passed passed, $failed failed"
if [ $failed -gt 0 ]; then
    echo -e "Failures:\n$errors"
    exit 1
fi
echo "All tests passed."
'
