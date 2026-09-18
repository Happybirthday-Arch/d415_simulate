"""Small, deterministic binary STL and solid-integral helpers (SI units)."""
import struct
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull

DTYPE = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])


def read_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n, = struct.unpack('<I', f.read(4))
        data = np.fromfile(f, dtype=DTYPE, count=n)
    if len(data) != n:
        raise ValueError(f'Truncated STL: {path}')
    return data['vertices'].astype(float)


def write_stl(path, triangles):
    t = np.asarray(triangles)
    data = np.zeros(len(t), dtype=DTYPE)
    data['vertices'] = t
    normal = np.cross(t[:, 1]-t[:, 0], t[:, 2]-t[:, 0])
    length = np.linalg.norm(normal, axis=1)
    good = length > 1e-18
    normal[good] /= length[good, None]
    data['normal'] = normal
    with open(path, 'wb') as f:
        f.write(b'Gazebo robot derived geometry, units meters'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(t)))
        data.tofile(f)


def split_mesh(t):
    vertices, inverse = np.unique(np.round(t.reshape(-1, 3), 8), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    graph = coo_matrix((np.ones(len(faces)*3),
                       (faces.ravel(), np.roll(faces, 1, axis=1).ravel())),
                      shape=(len(vertices), len(vertices)))
    n, labels = connected_components(graph, directed=False)
    return [t[labels[faces[:, 0]] == i] for i in range(n)]


def hull_mesh(t):
    vertices = np.unique(t.reshape(-1, 3), axis=0)
    hull = ConvexHull(vertices)
    result = vertices[hull.simplices].copy()
    normals = np.cross(result[:, 1]-result[:, 0], result[:, 2]-result[:, 0])
    reverse = np.sum(normals*hull.equations[:, :3], axis=1) < 0
    result[reverse] = result[reverse][:, [0, 2, 1]]
    return result


def mass_properties(t, mass):
    # Signed tetrahedra (origin, a, b, c). A local reference reduces cancellation.
    reference = t.reshape(-1, 3).mean(axis=0)
    p = t-reference
    volume = np.einsum('ij,ij->i', p[:, 0], np.cross(p[:, 1], p[:, 2]))/6
    total = volume.sum()
    if total <= 1e-15:
        raise ValueError('Nonpositive mesh volume')
    sums = p.sum(axis=1)
    center = np.sum(volume[:, None]*sums/4, axis=0)/total
    second = np.einsum('n,ni,nj->ij', volume, sums, sums)
    second += np.einsum('n,nki,nkj->ij', volume, p, p)
    second *= mass/(20*total)
    second -= mass*np.outer(center, center)
    inertia = np.trace(second)*np.eye(3)-second
    if np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError('Invalid inertia')
    return mass, center+reference, inertia


def combine(parts):
    mass = sum(p[0] for p in parts)
    center = sum(p[0]*p[1] for p in parts)/mass
    inertia = np.zeros((3, 3))
    for m, c, moment in parts:
        d = c-center
        inertia += moment+m*(np.dot(d, d)*np.eye(3)-np.outer(d, d))
    return mass, center, inertia
