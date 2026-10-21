import numpy as np
def components(tri, nv):
    # union-find over vertex ids via triangles -> island id per vertex
    parent=np.arange(nv)
    def find(a):
        while parent[a]!=a:
            parent[a]=parent[parent[a]]; a=parent[a]
        return a
    for a,b,c in tri:
        ra,rb,rc=find(a),find(b),find(c)
        if ra!=rb: parent[rb]=ra
        rc=find(c); ra=find(a)
        if rc!=ra: parent[rc]=ra
    return np.array([find(i) for i in range(nv)])
def cluster(pos, uv, tri, cell, island=None, nor=None):
    """Vertex clustering. Returns new pos, uv, tris. Corner key = (position cell, island) so UV seams survive."""
    mn=pos.min(0)
    q=np.floor((pos-mn)/cell).astype(np.int64)
    dims=q.max(0)+1
    ckey=(q[:,0]*dims[1]+q[:,1])*dims[2]+q[:,2]
    # cluster position = centroid of the vertices of the cell
    cu,cinv=np.unique(ckey,return_inverse=True)
    cp=np.zeros((len(cu),3)); cnt=np.bincount(cinv,minlength=len(cu))
    for k in range(3): cp[:,k]=np.bincount(cinv,weights=pos[:,k],minlength=len(cu))/cnt
    # drop degenerate triangles (two corners in one cell)
    ct=cinv[tri]
    ok=(ct[:,0]!=ct[:,1])&(ct[:,1]!=ct[:,2])&(ct[:,0]!=ct[:,2])
    tri=tri[ok]; ct=ct[ok]
    if island is None: island=np.zeros(len(pos),np.int64)
    # new vertex = (cluster, island)
    vk=cinv.astype(np.int64)*(island.max()+1)+island
    vu,vinv=np.unique(vk,return_inverse=True)
    newcl=np.zeros(len(vu),np.int64); newcl[vinv]=cinv
    used=np.unique(vinv[tri])
    remap=-np.ones(len(vu),np.int64); remap[used]=np.arange(len(used))
    ntri=remap[vinv[tri]]
    npos=cp[newcl[used]]
    if uv is not None:
        # uv: average of the original vertices of this (cluster, island)
        nuv=np.zeros((len(vu),2)); c2=np.bincount(vinv,minlength=len(vu))
        for k in range(2): nuv[:,k]=np.bincount(vinv,weights=uv[:,k],minlength=len(vu))/c2
        nuv=nuv[used]
    else: nuv=None
    if nor is not None:
        nn=np.zeros((len(vu),3)); 
        for k in range(3): nn[:,k]=np.bincount(vinv,weights=nor[:,k],minlength=len(vu))
        nn=nn[used]; l=np.linalg.norm(nn,axis=1,keepdims=True); nn=nn/np.maximum(l,1e-9)
    else: nn=None
    # remove duplicate triangles
    s=np.sort(ntri,axis=1); _,ui=np.unique(s,axis=0,return_index=True)
    return npos,nuv,ntri[np.sort(ui)],nn
def fit(pos,uv,tri,target,cell_max,island=None,nor=None):
    lo=cell_max/200.0
    r=cluster(pos,uv,tri,lo,island,nor)
    if len(r[2])<=target: return r
    best=None; a=lo; b=cell_max
    # exponential search up
    c=lo
    while c<cell_max:
        c=min(c*1.07,cell_max)
        r=cluster(pos,uv,tri,c,island,nor)
        if len(r[2])<=target: return r
        best=r
    return best
def normals(pos,tri):
    n=np.zeros_like(pos)
    f=np.cross(pos[tri[:,1]]-pos[tri[:,0]],pos[tri[:,2]]-pos[tri[:,0]])
    for k in range(3): np.add.at(n,tri[:,k],f)
    l=np.linalg.norm(n,axis=1,keepdims=True); n=n/np.maximum(l,1e-12)
    return n
