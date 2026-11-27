#!/usr/bin/env bash

# pull in system GPU driver libraries (no pixi package exists)

mkdir -p "$CONDA_PREFIX/lib/nvidia"

libs=(
    /usr/lib/x86_64-linux-gnu/libcuda.so* \
    /usr/lib/x86_64-linux-gnu/libnvidia-encode.so* \
    /usr/lib/x86_64-linux-gnu/libnvcuvid.so*
)

for lib in "${libs[@]}"; do
    [ -e "$lib" ] || continue
    ln -sf "$(readlink -f "$lib")" "$CONDA_PREFIX/lib/nvidia/$(basename "$lib")"
done

# libsl_zed.so expects librt.so
# create a linker script because librt is a part of libc, so librt.so is missing

cat > "$CONDA_PREFIX/x86_64-conda-linux-gnu/sysroot/usr/lib/librt.so" <<'EOF'
INPUT ( -lc )
EOF
