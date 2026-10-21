import struct,sys
def parse(path):
    d=open(path,'rb').read()
    p=d.index(b'\n')+1
    ver,endian,uv,nb=struct.unpack_from('<IBII',d,p);p+=13
    bsv,=struct.unpack_from('<I',d,p);p+=4
    for _ in range(4 if bsv==130 else 3):
        n=d[p];p+=1+n
    # max filepath for bsv==130 ? try: 
    ntypes,=struct.unpack_from('<H',d,p);p+=2
    types=[]
    for _ in range(ntypes):
        n,=struct.unpack_from('<I',d,p);p+=4;types.append(d[p:p+n].decode());p+=n
    idx=struct.unpack_from('<%dH'%nb,d,p);p+=2*nb
    sizes=struct.unpack_from('<%dI'%nb,d,p);p+=4*nb
    ns,=struct.unpack_from('<I',d,p);p+=4
    maxs,=struct.unpack_from('<I',d,p);p+=4
    strs=[]
    for _ in range(ns):
        n,=struct.unpack_from('<I',d,p);p+=4;strs.append(d[p:p+n].decode(errors='replace'));p+=n
    ng,=struct.unpack_from('<I',d,p);p+=4
    p+=4*ng
    blocks=[];off=p
    for i in range(nb):
        blocks.append((types[idx[i]],off,sizes[i]));off+=sizes[i]
    return d,bsv,strs,blocks,p
if __name__=='__main__':
    d,bsv,strs,blocks,p=parse(sys.argv[1])
    print('bsv',bsv,'datastart',p,'strings',len(strs))
    for i,s in enumerate(strs): print(i,s)
    for i,(t,o,s) in enumerate(blocks): print(i,t,o,s)
