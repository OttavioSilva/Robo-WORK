"""Interactive point checks in the front-edge reference; never navigation data."""
import numpy as np
import cv2
from .projection import to_ground


def inspect_point(pixel, calibration, fitting_pixels=()):
    matrix,size,polygon = calibration
    point = np.asarray(pixel,dtype=float)
    if point.shape != (2,) or not np.isfinite(point).all():
        return None
    if not (0 <= point[0] < size[0] and 0 <= point[1] < size[1]):
        return None
    if cv2.pointPolygonTest(polygon,tuple(point),False) < 0:
        return None
    h = to_ground(calibration, point)[0]
    if not np.isfinite(h).all() or abs(h[2]) < 1e-9:
        return None
    metres = h[:2]/h[2]
    if metres[0] < 0:
        return None
    fitting = np.asarray(fitting_pixels,dtype=float)
    reused = bool(len(fitting) and np.linalg.norm(fitting-point,axis=1).min() < 5.)
    return dict(pixel=point.tolist(),predicted_m=metres.tolist(),fitting_reference=reused)


def measured_check(selection, text):
    if selection is None:
        raise ValueError('Clique primeiro em um ponto dentro da regiao azul')
    if selection['fitting_reference']:
        raise ValueError('Cruz de ajuste: escolha um ponto novo para validar')
    try:
        values = [float(v.replace(',','.'))/100 for v in text.split()]
    except ValueError as error:
        raise ValueError('Digite dois numeros: frente e lateral em cm') from error
    if len(values) != 2 or not np.isfinite(values).all() or values[0] < 0:
        raise ValueError('Digite dois numeros: frente e lateral em cm')
    predicted = np.asarray(selection['predicted_m'])
    return dict(pixel=selection['pixel'],metres=values,predicted_m=predicted.tolist(),
                error_m=float(np.linalg.norm(predicted-values)))
