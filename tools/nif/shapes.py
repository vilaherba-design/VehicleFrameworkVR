import struct,sys,numpy as np
from nifhdr import parse
def shape(d,strs,off,size):
    p=off
    ni,ne=struct.unpack_from('<iI',d,p);p+=8
    p+=4*ne+4
    flags,=struct.unpack_from('<I',d,p);p+=4
    tr=struct.unpack_from('<3f',d,p);p+=12
    rot=np.array(struct.unpack_from('<9f',d,p)).reshape(3,3);p+=36
    sc,=struct.unpack_from('<f',d,p);p+=4
    p+=4 # collision
    bs=struct.unpack_from('<4f',d,p);p+=16
    skin,shader,alpha=struct.unpack_from('<3i',d,p);p+=12
    vd,=struct.unpack_from('<Q',d,p);p+=8
    nt,nv,ds=struct.unpack_from('<IHI',d,p) if False else (None,None,None)
    return dict(name=strs[ni] if ni>=0 else None,tr=tr,rot=rot,sc=sc,bs=bs,shader=shader,alpha=alpha,vd=vd,p=p,end=off+size)
def load(path):
    d,bsv,strs,blocks,_=parse(path)
    out=[]
    for i,(t,o,s) in enumerate(blocks):
        if t.startswith('BS') and 'TriShape' in t:
            h=shape(d,strs,o,s)
            p=h['p']
            nt,=struct.unpack_from('<I',d,p);p+=4
            nv,=struct.unpack_from('<H',d,p);p+=2 if False else 0
            # layout: numTriangles u32, numVertices u16, dataSize u32
            nv,=struct.unpack_from('<H',d,p);p+=2
            dsz,=struct.unpack_from('<I',d,p);p+=4
            vsz=(h['vd']&0xF)*4
            vb=np.frombuffer(d,dtype=np.uint8,count=nv*vsz,offset=p).reshape(nv,vsz)
            full=bool((h['vd']>>44)&0x400)
            if full:
                pos=np.frombuffer(vb[:,0:12].tobytes(),dtype=np.float32).reshape(nv,3).copy()
            else:
                pos=np.frombuffer(np.ascontiguousarray(vb[:,0:6]).tobytes(),dtype=np.float16).reshape(nv,3).astype(np.float32)
            p+=nv*vsz
            tri=np.frombuffer(d,dtype=np.uint16,count=nt*3,offset=p).reshape(nt,3);p+=nt*6
            h.update(idx=i,nt=nt,nv=nv,vsz=vsz,pos=pos,tri=tri,after=p,type=t)
            out.append(h)
    return d,bsv,strs,blocks,out
if __name__=='__main__':
    d,bsv,strs,blocks,sh=load(sys.argv[1])
    for h in sh:
        P=h['pos']
        print(h['idx'],h['name'],'nt',h['nt'],'nv',h['nv'],'vsz',h['vsz'],'vd %x'%h['vd'],'rem',h['end']-h['after'])
        print('  tr',h['tr'],'min',P.min(0),'max',P.max(0))
