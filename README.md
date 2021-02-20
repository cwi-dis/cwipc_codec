> Copyright (c) 2017-2021, Stichting Centrum Wiskunde en Informatica (CWI).

# cwipc_codec

This distribution contains a compressor and decompressor for pointclouds. It uses the _cwipc_ abstract object to represent pointclouds.

It is a modified version of the cwi-pcl-codec distribution, from <http://github.com/cwi-dis/cwi-pcl-codec>. Both distributions share the same codec, described in the following journal paper:

> _(R. Mekuria, K. Blom, and P. Cesar, "Design, Implementation and Evaluation of a Point Cloud Codec for Tele-Immersive Video," IEEE Transactions on Circuits and Systems for Video Technology, 27(4): 828 -842, 2017_

## Installing

For use within VRtogether you can get pre-built zipfiles (or tgzfiles for Mac/Linux) from <https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_codec/releases>. Download the most recent release with a normal v_X_._Y_._Z_ name. You will also need the accompanying _cwipc\_util_ installer from 
<https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util/releases>.

You also need to install a number of dependencies:

* PCL, and its dependencies:
	* Boost
	* Eigen
	* Flann
	* QHull
* libjpeg-turbo

[![pipeline status](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_codec/badges/master/pipeline.svg)](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_codec/commits/master)

### Windows

- Install `cwipc_util` and its dependencies (including PCL).
- Download libjpeg-turbo via <https://libjpeg-turbo.org/>. Get the windows 64bit binary installer. Install in the standard location `C:\libjpeg-turbo64`. Add `C:\libjpeg-turbo64\bin` to %PATH% for all users.
- Extract both the `cwipc_util_win1064_vX.Y.zip` and `cwipc_codec_win1064_vX.Y.zip` files into `c:\vrtogether`. This will create `bin`, `lib` and `include` folders inside the `C:\vrtogether\installed` folder.
- Add the `c:\vrtogether\installed\bin` folder to the `%PATH%` system environment variable.

### OSX

- Install _brew_, and then

  ```
  brew install pcl
  brew install jpeg-turbo
  brew unlink jpeg
  brew link --force jpeg-turbo
  ```

- Extract both gzip files into `/usr/local`:

  ```
  cd /usr/local
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  [sudo] tar xfv .../cwipc_codec_osx1012_vX.Y.tgz
  ```
  
### Ubuntu 20.04

- Install _PCL_ and _jpegturbo_ with 

  ```
  apt-get install libpcl-dev
  sudo apt-get install libturbojpeg0-dev
  ```

- Extract both gzip files into `/usr/local`:

  ```
  cd /usr/local
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  [sudo] tar xfv .../cwipc_codec_osx1012_vX.Y.tgz
  ```

## Building from source

It is expected that if you build cwipc_codec from source you also build cwipc_util from source. Therefore, the instructions here assume you have already followed the instructions for building cwipc_util.

### OSX instructions

- Build and install cwipc_util.
- Install a few dependencies needed by our software and some of the third party libraries:

  ```
  brew install jpeg-turbo
  brew unlink jpeg
  brew link --force jpeg-turbo
  ```
  
  - The `brew unlink` and `brew link` are needed to install *jpeg-turbo* in stead of the normal jpeg library. Brew prefers not to do this, but that may lead to problems with some parts of the program linking to normal libjpeg and others to jpeg-turbo and the two getting into each others way.
- Build cwipc_codec by following the same instructions as for cwipc_util.
	
### Linux instructions

- Build and install cwipc_util.
- Install a few dependencies needed by our software and some of the third party libraries:

  ```
  sudo apt-get install libturbojpeg0-dev
  ```
  

- Build cwipc_codec by following the same instructions as for cwipc_util.


### Windows instructions

  - Build and install cwipc_util.
  - Download libjpeg-turbo via <https://libjpeg-turbo.org/>. Get the windows 64bit binary installer. Install in the standard location `C:\libjpeg-turbo64`. Add `C:\libjpeg-turbo64\bin` to %PATH% for all users.
  - Build cwipc_codec, with a slightly different cmake command than for cwipc_util because cmake needs help finding libjpeg-turbo:
  
  ```
  instdir=`pwd`/../installed
  mkdir -p build
  cd build
  cmake .. -G "Visual Studio 16 2019" -DCMAKE_INSTALL_PREFIX="$instdir" -DJPEG_Turbo_INCLUDE_DIR="C:/libjpeg-turbo64/include" -DJPEG_Turbo_LIBRARY="C:/libjpeg-turbo64/lib/jpeg.lib"
  cmake --build . --config Release
  cmake --build . --config Release --target RUN_TESTS
  cmake --build . --config Release --target INSTALL

  ```

## Test programs

Three test programs are included:

- `cwipc_encode` compresses a `.ply` pointcloud file to a `.cwicpc` compressed pointcloud file.
- `cwipc_decode` decompresses a `.cwicpc` compresssed file to a `.ply` pointcloud file.
- `evaluate_compression` allows for evaluation of the algorithm, see [its readme file](apps/evaluate_compression/readme.md) for details.

The Python unittest [python/test\_cwipc\_codec.py](python/test_cwipc_codec.py) tests each individual feature and API call of the codec.