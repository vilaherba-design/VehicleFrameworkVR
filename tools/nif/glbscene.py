import sys; sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import glb, numpy as np
def qmat(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def local(n):
    M=np.eye(4)
    if 'matrix' in n: return np.array(n['matrix']).reshape(4,4).T
    R=qmat(n['rotation']) if 'rotation' in n else np.eye(3)
    S=np.diag(n['scale']) if 'scale' in n else np.eye(3)
    M[:3,:3]=R@S
    if 'translation' in n: M[:3,3]=n['translation']
    return M
def prims(path):
    js,b=glb.load(path)
    out=[]
    def walk(i,P,path_):
        n=js['nodes'][i]; M=P@local(n); nm=path_+'/'+n.get('name','')
        if 'mesh' in n:
            for pr in js['meshes'][n['mesh']]['primitives']:
                pos=glb.accessor(js,b,pr['attributes']['POSITION']).astype(np.float64)
                nor=glb.accessor(js,b,pr['attributes']['NORMAL']).astype(np.float64) if 'NORMAL' in pr['attributes'] else None
                uv=glb.accessor(js,b,pr['attributes']['TEXCOORD_0']).astype(np.float64) if 'TEXCOORD_0' in pr['attributes'] else None
                idx=(glb.accessor(js,b,pr['indices']).reshape(-1,3).astype(np.int64) if 'indices' in pr else np.arange(len(pos)).reshape(-1,3))
                if 'indices' not in pr:
                    # unindexed mesh: weld corners that agree on position, normal and uv
                    key=np.round(np.concatenate([pos,nor if nor is not None else np.zeros((len(pos),3)),uv if uv is not None else np.zeros((len(pos),2))],axis=1),5)
                    _,first,inv=np.unique(key,axis=0,return_index=True,return_inverse=True)
                    pos=pos[first]; nor=nor[first] if nor is not None else None; uv=uv[first] if uv is not None else None
                    idx=inv.reshape(-1)[idx.reshape(-1)].reshape(-1,3)
                wp=pos@M[:3,:3].T+M[:3,3]
                wn=None
                if nor is not None:
                    wn=nor@np.linalg.inv(M[:3,:3]).T; wn/=np.linalg.norm(wn,axis=1,keepdims=True)+1e-12
                out.append(dict(name=n.get('name',''),path=nm,mat=js['materials'][pr['material']]['name'],matidx=pr['material'],pos=wp,nor=wn,uv=uv,tri=idx))
        for c in n.get('children',[]): walk(c,M,nm)
    for r in js['scenes'][js['scene']]['nodes']: walk(r,np.eye(4),'')
    return js,b,out
