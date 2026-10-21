import json,struct,numpy as np
def load(path):
    d=open(path,'rb').read()
    magic,ver,length=struct.unpack_from('<4sII',d,0)
    p=12; js=None; binc=None
    while p<len(d):
        cl,ct=struct.unpack_from('<II',d,p); p+=8
        chunk=d[p:p+cl]; p+=cl
        if ct==0x4E4F534A: js=json.loads(chunk)
        elif ct==0x004E4942: binc=chunk
    return js,binc
def accessor(js,binc,i):
    a=js['accessors'][i]; bv=js['bufferViews'][a['bufferView']]
    comp={5120:np.int8,5121:np.uint8,5122:np.int16,5123:np.uint16,5125:np.uint32,5126:np.float32}[a['componentType']]
    n={'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4,'MAT4':16}[a['type']]
    off=bv.get('byteOffset',0)+a.get('byteOffset',0)
    stride=bv.get('byteStride',0)
    isz=np.dtype(comp).itemsize*n
    if stride and stride!=isz:
        raw=np.frombuffer(binc,dtype=np.uint8,count=stride*a['count'],offset=off).reshape(a['count'],stride)[:,:isz].copy()
        return raw.view(comp).reshape(a['count'],n)
    return np.frombuffer(binc,dtype=comp,count=a['count']*n,offset=off).reshape(a['count'],n)
