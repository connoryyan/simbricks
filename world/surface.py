from dataclasses import dataclass


@dataclass(frozen=True)
class SurfaceHit:
    '''
    Store a surface hit in world coordinates.
    '''
    x_mm: float
    y_mm: float
    z_mm: float
    distance_mm: float
    reflection: int
    incidence_cosine: float


