#!/bin/bash
if [ "$(arch)" == "aarch64" ]
then
    echo $(dirname "${0}")/glsl2spirvarm "$@"
    $(dirname "${0}")/glsl2spirvarm "$@"
else
    echo $(dirname "${0}")/glsl2spirv "$@"
    $(dirname "${0}")/glsl2spirv "$@"
fi

