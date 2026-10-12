# Shared by body/tools/host_tests.sh, body/tools/build_link_harness.sh and
# protocol/test.sh (sourced, not run).

# libasan on 32-bit ARM (Raspberry Pi, armhf) needs libatomic for its 64-bit
# atomics (__atomic_load_8/__atomic_store_8). Probe whether a plain sanitizer
# link works and, if not, print -latomic for the caller's link line.
# Usage: kilo_sanitizer_ldlibs CXX SCRATCH_DIR
kilo_sanitizer_ldlibs() {
  local cxx="$1" dir="$2"
  printf 'int main(){return 0;}\n' > "$dir/probe.cpp"
  if ! "$cxx" -fsanitize=address,undefined "$dir/probe.cpp" -o "$dir/probe" 2>/dev/null &&
     "$cxx" -fsanitize=address,undefined "$dir/probe.cpp" -o "$dir/probe" -latomic 2>/dev/null; then
    echo "(sanitizer link needs -latomic here)" >&2
    echo "-latomic"
  fi
}

# The host build of the body core: every core/ source plus the fakes for its
# EEPROM and link HAL seams. The protocol C files (src/proto/*.c) are C99 and
# each caller compiles them itself (with or without sanitizers).
# Usage: kilo_core_sources BODY_DIR   (sets the array KILO_CORE_SRCS)
kilo_core_sources() {
  KILO_CORE_SRCS=("$1"/src/core/*.cpp "$1/test/fake_eeprom.cpp" "$1/test/fake_link_hal.cpp")
}
