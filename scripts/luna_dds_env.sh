#!/usr/bin/env bash
DDS_DEPS="${XDG_CACHE_HOME:-$HOME/.cache}/head_2dof_capt/luna-dds-preview-deps"
if [[ ! -d "$DDS_DEPS/cyclonedds" ]]; then
    echo "Installing CycloneDDS Python bindings in $DDS_DEPS"
    python3 -m pip install --target "$DDS_DEPS" cyclonedds
fi
export PYTHONPATH="$DDS_DEPS${PYTHONPATH:+:$PYTHONPATH}"
