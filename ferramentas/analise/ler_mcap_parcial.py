"""Le MCAP somente ate o tamanho inicial, inclusive gravacao aberta; nunca reindexa o original.
Formato dos registros conforme mcap_vendor/mcap/types.hpp. Ignora apenas cauda incompleta.
"""
import ctypes,ctypes.util,io,struct,zlib
from pathlib import Path


def registros(f,limite):
    while f.tell()+9 <= limite:
        cab=f.read(9)
        if len(cab)!=9:return
        op,n=struct.unpack('<BQ',cab)
        if n>limite-f.tell():return
        body=f.read(n)
        if len(body)!=n:return
        yield op,body


def string(b,pos):
    n=struct.unpack_from('<I',b,pos)[0];pos+=4
    return b[pos:pos+n].decode(),pos+n


def mensagens(path,topicos=None):
    path=Path(path);limit=path.stat().st_size;schemas={};channels={};zstd=None
    with path.open('rb') as f:
        if f.read(8)!=b'\x89MCAP0\r\n':raise ValueError('magic MCAP invalido')
        def decode(records):
            nonlocal zstd
            for op,b in records:
                if op==3:
                    sid=struct.unpack_from('<H',b)[0];name,_=string(b,2);schemas[sid]=name
                elif op==4:
                    cid,sid=struct.unpack_from('<HH',b);topic,_=string(b,4);channels[cid]=(topic,sid)
                elif op==5:
                    cid,seq,t,pub=struct.unpack_from('<HIQQ',b);topic,sid=channels[cid]
                    if topicos is None or topic in topicos:yield topic,schemas[sid],t,b[22:]
                elif op==6:
                    size=struct.unpack_from('<Q',b,16)[0];crc=struct.unpack_from('<I',b,24)[0]
                    comp,pos=string(b,28);n=struct.unpack_from('<Q',b,pos)[0];data=b[pos+8:pos+8+n]
                    if comp=='zstd':
                        if zstd is None:
                            zstd=ctypes.CDLL(ctypes.util.find_library('zstd'))
                            zstd.ZSTD_decompress.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t]
                            zstd.ZSTD_decompress.restype=ctypes.c_size_t
                        dst=ctypes.create_string_buffer(size);got=zstd.ZSTD_decompress(dst,size,data,len(data))
                        if got!=size:raise ValueError('chunk zstd invalido')
                        data=dst.raw
                    elif comp=='lz4':
                        import lz4.frame
                        data=lz4.frame.decompress(data)
                    elif comp:raise ValueError('compressao desconhecida: '+comp)
                    if len(data)!=size or (crc and zlib.crc32(data)!=crc):raise ValueError('CRC/tamanho chunk invalido')
                    yield from decode(registros(io.BytesIO(data),len(data)))
        yield from decode(registros(f,limit))
