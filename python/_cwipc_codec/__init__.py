import os
import ctypes
import ctypes.util
import warnings
from typing import Optional
from cwipc.util import CwipcError, CWIPC_API_VERSION, cwipc, cwipc_source, cwipc_point, cwipc_point_array
from cwipc.util import cwipc_p, cwipc_source_p
from cwipc.util import _cwipc_dll_search_path_collection # type: ignore

#
# This is a workaround for the change in DLL loading semantics on Windows since Python 3.8
# Python no longer uses the PATH environment variable to load dependent dlls but only
# its own set. For that reason we list here a set of dependencies that we know are needed,
# find those on PATH, and add the directories where those DLLs are located while loading our
# DLL.
# The list does not have to be complete, as long as at least one DLL from each directory needed
# is listed.
# Dependencies of cwipc_util are automatically added.
# NOTE: this list must be kept up-to-date otherwise loading DLLs will fail with
# an obscure message "Python could not find module .... or one of its dependencies"
#
_WINDOWS_NEEDED_DLLS=[ # NOT USED AT THE TIME. CAUSING DLL Loading problems
    "turbojpeg",
    "jpeg62"
]

class cwipc_encoder_p(ctypes.c_void_p):
    pass
    
class cwipc_encodergroup_p(ctypes.c_void_p):
    pass
    
class cwipc_decoder_p(cwipc_source_p):
    pass
    
_cwipc_codec_dll_reference = None

#
# NOTE: the signatures here must match those in cwipc_util/api.h or all hell will break loose
#
def cwipc_codec_dll_load(libname : Optional[str]=None) -> ctypes.CDLL:
    """Load the cwipc_util DLL and assign the signatures (if not already loaded)"""
    global _cwipc_codec_dll_reference
    if _cwipc_codec_dll_reference: return _cwipc_codec_dll_reference
    
    with _cwipc_dll_search_path_collection(None) as loader:
        if libname == None:
            libname = 'cwipc_codec'
        if not os.path.isabs(libname):
            libname = loader.find_library(libname)
            if not libname:
                raise RuntimeError('Dynamic library cwipc_codec not found')
        assert libname
        _cwipc_codec_dll_reference = ctypes.CDLL(libname)
        if not _cwipc_codec_dll_reference:
            raise RuntimeError(f'Dynamic library {libname} cannot be loaded')
    

    _cwipc_codec_dll_reference.cwipc_new_encoder.argtypes = [ctypes.c_int, ctypes.POINTER(cwipc_encoder_params), ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_codec_dll_reference.cwipc_new_encoder.restype = cwipc_encoder_p
    _cwipc_codec_dll_reference.cwipc_encoder_free.argtypes = [cwipc_encoder_p]
    _cwipc_codec_dll_reference.cwipc_encoder_free.restype = None
    _cwipc_codec_dll_reference.cwipc_encoder_available.argtypes = [cwipc_encoder_p, ctypes.c_bool]
    _cwipc_codec_dll_reference.cwipc_encoder_available.restype = ctypes.c_bool
    _cwipc_codec_dll_reference.cwipc_encoder_eof.argtypes = [cwipc_encoder_p]
    _cwipc_codec_dll_reference.cwipc_encoder_eof.restype = ctypes.c_bool
    _cwipc_codec_dll_reference.cwipc_encoder_at_gop_boundary.argtypes = [cwipc_encoder_p]
    _cwipc_codec_dll_reference.cwipc_encoder_at_gop_boundary.restype = ctypes.c_bool
    _cwipc_codec_dll_reference.cwipc_encoder_feed.argtypes = [cwipc_encoder_p, cwipc_p]
    _cwipc_codec_dll_reference.cwipc_encoder_feed.restype = None
    _cwipc_codec_dll_reference.cwipc_encoder_get_encoded_size.argtypes = [cwipc_encoder_p]
    _cwipc_codec_dll_reference.cwipc_encoder_get_encoded_size.restype = ctypes.c_size_t
    _cwipc_codec_dll_reference.cwipc_encoder_copy_data.argtypes = [cwipc_encoder_p, ctypes.c_void_p, ctypes.c_size_t]
    _cwipc_codec_dll_reference.cwipc_encoder_copy_data.restype = ctypes.c_bool
    if hasattr(_cwipc_codec_dll_reference, 'cwipc_encoder_close'):
        _cwipc_codec_dll_reference.cwipc_encoder_close.argtypes = [cwipc_encoder_p]
        _cwipc_codec_dll_reference.cwipc_encoder_close.restype = None

    _cwipc_codec_dll_reference.cwipc_new_encodergroup.argtypes = [ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_codec_dll_reference.cwipc_new_encodergroup.restype = cwipc_encodergroup_p
    _cwipc_codec_dll_reference.cwipc_encodergroup_addencoder.argtypes = [cwipc_encodergroup_p, ctypes.c_int, ctypes.POINTER(cwipc_encoder_params), ctypes.POINTER(ctypes.c_char_p)]
    _cwipc_codec_dll_reference.cwipc_encodergroup_addencoder.restype = cwipc_encoder_p
    _cwipc_codec_dll_reference.cwipc_encodergroup_feed.argtypes = [cwipc_encodergroup_p, cwipc_p]
    _cwipc_codec_dll_reference.cwipc_encodergroup_feed.restype = None
    if hasattr(_cwipc_codec_dll_reference, 'cwipc_encodergroup_close'):
        _cwipc_codec_dll_reference.cwipc_encodergroup_close.argtypes = [cwipc_encodergroup_p]
        _cwipc_codec_dll_reference.cwipc_encodergroup_close.restype = None

    _cwipc_codec_dll_reference.cwipc_new_decoder.argtypes = [ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_codec_dll_reference.cwipc_new_decoder.restype = cwipc_decoder_p
    _cwipc_codec_dll_reference.cwipc_decoder_feed.argtypes = [cwipc_decoder_p, ctypes.c_void_p, ctypes.c_size_t]
    _cwipc_codec_dll_reference.cwipc_decoder_feed.restype = None
    if hasattr(_cwipc_codec_dll_reference, 'cwipc_decoder_close'):
        _cwipc_codec_dll_reference.cwipc_decoder_close.argtypes = [cwipc_decoder_p]
        _cwipc_codec_dll_reference.cwipc_decoder_close.restype = None



    return _cwipc_codec_dll_reference

class cwipc_encoder_params(ctypes.Structure):
    """Parameters to control cwipc compression"""
    _fields_ = [
        ("do_inter_frame", ctypes.c_bool),
        ("gop_size", ctypes.c_int),
        ("exp_factor", ctypes.c_float),
        ("octree_bits", ctypes.c_int),
        ("jpeg_quality", ctypes.c_int),
        ("macroblock_size", ctypes.c_int),
        ("tilenumber", ctypes.c_int),
        ("voxelsize", ctypes.c_float),
        ("n_parallel", ctypes.c_int),
        ]
CWIPC_ENCODER_PARAM_VERSION = 0x20220607

class cwipc_encoder_wrapper:
    _cwipc_encoder : Optional[cwipc_encoder_p]

    def __init__(self, _cwipc_encoder : Optional[cwipc_encoder_p]):
        if _cwipc_encoder != None:
            assert isinstance(_cwipc_encoder, cwipc_encoder_p)
        self._cwipc_encoder = _cwipc_encoder
        
    def _as_cwipc_encoder_p(self) -> cwipc_encoder_p:
        assert self._cwipc_encoder
        return self._cwipc_encoder
        
    def free(self) -> None:
        if self._cwipc_encoder:
            cwipc_codec_dll_load().cwipc_encoder_free(self._as_cwipc_encoder_p())
        self._cwipc_encoder = None

    def close(self) -> None:
        if self._cwipc_encoder:
            cwipc_codec_dll_load().cwipc_encoder_close(self._as_cwipc_encoder_p())
        
    def eof(self) -> bool:
        rv = cwipc_codec_dll_load().cwipc_encoder_eof(self._as_cwipc_encoder_p())
        return rv
        
    def at_gop_boundary(self) -> bool:
        rv = cwipc_codec_dll_load().cwipc_encoder_at_gop_boundary(self._as_cwipc_encoder_p())
        return rv
        
    def available(self, wait : bool) -> bool:
        rv = cwipc_codec_dll_load().cwipc_encoder_available(self._as_cwipc_encoder_p(), wait)
        return rv
        
    def feed(self, pc: cwipc) -> None:
        rv = cwipc_codec_dll_load().cwipc_encoder_feed(self._as_cwipc_encoder_p(), pc._as_cwipc_p())
        return rv
        
    def get_encoded_size(self) -> int:
        rv = cwipc_codec_dll_load().cwipc_encoder_get_encoded_size(self._as_cwipc_encoder_p())
        return rv
        
    def get_bytes(self) -> Optional[bytearray]:
        length = self.get_encoded_size()
        rv = bytearray(length)
        ptr_char = (ctypes.c_char * length).from_buffer(rv)
        ptr = ctypes.cast(ptr_char, ctypes.c_void_p)
        ok = cwipc_codec_dll_load().cwipc_encoder_copy_data(self._as_cwipc_encoder_p(), ptr, length)
        if not ok:
            return None
        return rv
        
class cwipc_encodergroup_wrapper:
    _cwipc_encodergroup : Optional[cwipc_encodergroup_p]
    def __init__(self, _cwipc_encodergroup : Optional[cwipc_encodergroup_p]):
        if _cwipc_encodergroup != None:
            assert isinstance(_cwipc_encodergroup, cwipc_encodergroup_p)
        self._cwipc_encodergroup = _cwipc_encodergroup
        
    def _as_cwipc_encodergroup_p(self) -> cwipc_encodergroup_p:
        assert self._cwipc_encodergroup
        return self._cwipc_encodergroup
        
    def free(self):
        if self._cwipc_encodergroup:
            cwipc_codec_dll_load().cwipc_encodergroup_free(self._as_cwipc_encodergroup_p())
        self._cwipc_encodergroup = None

    def close(self):
        if self._cwipc_encodergroup:
            cwipc_codec_dll_load().cwipc_encodergroup_close(self._as_cwipc_encodergroup_p())

    def feed(self, pc : cwipc) -> None:
        rv = cwipc_codec_dll_load().cwipc_encodergroup_feed(self._as_cwipc_encodergroup_p(), pc.as_cwipc_p())
        return rv
        
    def addencoder(self, version=None, params=None, **kwargs):
        if version == None:
            version = CWIPC_ENCODER_PARAM_VERSION
        if isinstance(params, cwipc_encoder_params):
            pass
        else:
            params = cwipc_new_encoder_params(**kwargs)
        errorString = ctypes.c_char_p()
        obj = cwipc_codec_dll_load().cwipc_encodergroup_addencoder(self._as_cwipc_encodergroup_p(), version, params, ctypes.byref(errorString))
        if errorString and errorString.value and not rv:
            raise CwipcError(errorString.value.decode('utf8'))
        if errorString and errorString.value:
            warnings.warn(errorString.value.decode('utf8'))
        if not obj:
            return None
        return cwipc_encoder_wrapper(obj)

        
class cwipc_decoder_wrapper(cwipc_source):
    def __init__(self, _cwipc_decoder : Optional[cwipc_decoder_p]):
        if _cwipc_decoder != None:
            assert isinstance(_cwipc_decoder, cwipc_decoder_p)
        cwipc_source.__init__(self, _cwipc_decoder)
        
    def _as_cwipc_decoder_p(self) -> cwipc_source_p:
        assert self._cwipc_source
        return self._cwipc_source
        
    def feed(self, buffer) -> None:
        length = len(buffer)
        if isinstance(buffer, bytearray):
            buffer = (ctypes.c_char * length).from_buffer(buffer)
        ptr = ctypes.cast(buffer, ctypes.c_void_p)
        rv = cwipc_codec_dll_load().cwipc_decoder_feed(self._as_cwipc_decoder_p(), ptr, length)
        return rv

    def close(self):
        if self._cwipc_source:
            cwipc_codec_dll_load().cwipc_decoder_close(self._as_cwipc_decoder_p())

def cwipc_new_encoder_params(**kwargs):
    params = cwipc_encoder_params(False, 1, 1, 9, 85, 16, 0, 0, 0)
    for k, v in kwargs.items():
        assert hasattr(params, k), 'No encoder_param named {}'.format(k)
        setattr(params, k, v)
    return params

def cwipc_new_encoder(version=None, params=None, **kwargs):
    if version == None:
        version = CWIPC_ENCODER_PARAM_VERSION
    if isinstance(params, cwipc_encoder_params):
        pass
    else:
        params = cwipc_new_encoder_params(**kwargs)
    errorString = ctypes.c_char_p()
    obj = cwipc_codec_dll_load().cwipc_new_encoder(version, params, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and not obj:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString:
        warnings.warn(errorString.value.decode('utf8'))
    if not obj:
        return None
    return cwipc_encoder_wrapper(obj)
    
def cwipc_new_encodergroup():
    errorString = ctypes.c_char_p()
    obj = cwipc_codec_dll_load().cwipc_new_encodergroup(ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString:
        warnings.warn(errorString.value.decode('utf8'))
    if not obj:
        return None
    return cwipc_encodergroup_wrapper(obj)
    
def cwipc_new_decoder():
    errorString = ctypes.c_char_p()
    obj = cwipc_codec_dll_load().cwipc_new_decoder(ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString:
        warnings.warn(errorString.value.decode('utf8'))
    if not obj:
        return None
    return cwipc_decoder_wrapper(obj)
