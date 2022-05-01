> Copyright (c) 2017-2021, Stichting Centrum Wiskunde en Informatica (CWI).

This repository is a compression library for `cwipc` pointcloud objects. It is part of the `cwipc` suite, <https://github.com/cwi-dis/cwipc>, and should generally be installed or built as part of that suite. 

# cwipc_codec

This distribution contains a compressor and decompressor for pointclouds. It uses the _cwipc_ abstract object to represent pointclouds.

It is a modified version of the cwi-pcl-codec distribution, from <http://github.com/cwi-dis/cwi-pcl-codec>. Both distributions share the same codec, described in the following journal paper:

> _(R. Mekuria, K. Blom, and P. Cesar, "Design, Implementation and Evaluation of a Point Cloud Codec for Tele-Immersive Video," IEEE Transactions on Circuits and Systems for Video Technology, 27(4): 828 -842, 2017_

When cwipc_codec is built from source (as part of the _cwipc_ suite) it is possible to use the codec using the same API as specified in that paper and in the _cwi-pcl-codec_ repository. Simply add the include (and possibly source) directory to your project.

## Use

- `cwipc_encode` compresses a `.ply` pointcloud file to a `.cwicpc` compressed pointcloud file.
- `cwipc_decode` decompresses a `.cwicpc` compressed file to a `.ply` pointcloud file.
- `cwipc_grab --compress` will compress pointclouds before saving them to disk.
- `cwipc_view --playback` will decompress `.cwicpc` files on the fly.
- The various tools that produce or consume pointcloud streams will use the encoder and decoder by default.

Check `include/cwipc_codec/api.h` to see how to use the codec from your code. The Python API is nearly identical, inspect the `cwipc.codec` module.

## Evaluation program

When building from source there is a test program included:

- `evaluate_compression` allows for evaluation of the algorithm, see [its readme file](apps/evaluate_compression/readme.md) for details.
