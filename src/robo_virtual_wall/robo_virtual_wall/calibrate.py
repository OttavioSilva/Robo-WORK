"""Promote a front-reference preview only after independent measured checks."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from .projection import load_calibration, calibration_from_dict, to_ground


def validated_base_calibration(preview, checks, offset_x, offset_y, max_error=.02):
    """checks: pixel centres and independently measured front-reference metres.

    Coordinates use x forward, y left in both references; only origin translation
    is permitted. This does not estimate or compensate lens distortion.
    """
    if not np.isfinite([offset_x,offset_y,max_error]).all() or not 0 < max_error <= .03:
        raise ValueError('Invalid measured offsets or validation tolerance')
    if preview.get('frame_id') != 'virtual_wall_front' or preview.get('image_space') not in ('raw', 'undistorted'):
        raise ValueError('Expected raw-image front preview')
    # Raw previews keep the old minimal contract; lens previews are fully checked.
    model = (calibration_from_dict(preview, preview=True)
             if preview.get('image_space') == 'undistorted' else (np.asarray(preview['H'], float),))
    if 'image_size' in checks and checks['image_size'] != preview.get('image_size'):
        raise ValueError('Checks and calibration image resolutions differ')
    pixels = np.asarray(checks['pixels'],dtype=float)
    metres = np.asarray(checks['metres'],dtype=float)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) < 3 or metres.shape != pixels.shape:
        raise ValueError('At least three independent pixel/metre pairs are required')
    if not np.isfinite(pixels).all() or not np.isfinite(metres).all():
        raise ValueError('Non-finite checks')
    if np.ptp(metres[:,0]) < .10 or np.ptp(metres[:,1]) < .15:
        raise ValueError('Checks must span at least 10 cm forward and 15 cm laterally')
    if np.linalg.matrix_rank(metres-metres.mean(axis=0),tol=1e-6) < 2:
        raise ValueError('Checks must not be collinear')
    polygon = np.asarray(preview['valid_pixels'],dtype=np.float32)
    if any(cv2.pointPolygonTest(polygon,tuple(p),False)<0 for p in pixels):
        raise ValueError('Validation points outside calibrated region')
    fitting_pixels = np.asarray(preview['pixels'],dtype=float)
    if np.linalg.norm(pixels[:,None,:]-fitting_pixels[None,:,:],axis=2).min() < 3:
        raise ValueError('Validation must not reuse fitting crosses')
    H = np.asarray(preview['H'],dtype=float)
    homogeneous = to_ground(model, pixels)
    if (np.abs(homogeneous[:,2]) < 1e-9).any():
        raise ValueError('Invalid projection at validation points')
    predicted = homogeneous[:,:2]/homogeneous[:,2,None]
    errors = np.linalg.norm(predicted-metres,axis=1)
    if not np.isfinite(errors).all() or errors.max() > max_error:
        raise ValueError('Independent metric error exceeds tolerance: '+str(errors.tolist()))
    transform = np.array([[1,0,offset_x],[0,1,offset_y],[0,0,1.]])
    result = dict(preview)
    result.update(H=(transform @ H).tolist(),frame_id='base_link',validated=True,
                  metres=(np.asarray(preview['metres'])+[offset_x,offset_y]).tolist(),
                  front_offset_base_link_m=[offset_x,offset_y],
                  validation=dict(front_reference_checks=checks,errors_m=errors.tolist(),
                                  max_error_m=max_error))
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',required=True,type=Path)
    parser.add_argument('--checks',required=True,type=Path)
    parser.add_argument('--front-x',required=True,type=float,help='Measured front origin x in base_link, metres')
    parser.add_argument('--front-y',required=True,type=float,help='Measured front origin y in base_link, metres')
    parser.add_argument('--output',required=True,type=Path)
    options = parser.parse_args(args)
    load_calibration(options.preview,preview=True)
    result = validated_base_calibration(json.loads(options.preview.read_text()),
                 json.loads(options.checks.read_text()), options.front_x,options.front_y)
    # Never silently replace an existing calibration.
    with options.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2)
        stream.write('\n')
    print('Saved validated base_link calibration:',options.output)


def base_calibration_from_field_checks(preview, evidence, offset_x, offset_y, max_error=.02):
    """Translate an existing homography after documented manual field checks.

    This path records screen readings and user-confirmed tape measurements,
    not pixel correspondences or a refit. Its limited validation scope is retained.
    """
    if not np.isfinite([offset_x,offset_y,max_error]).all() or not 0 < max_error <= .03:
        raise ValueError('Invalid measured offsets or tolerance')
    if preview.get('frame_id') != 'virtual_wall_front' or preview.get('image_space') != 'raw':
        raise ValueError('Expected raw front-reference preview')
    if evidence.get('image_size') != preview.get('image_size'):
        raise ValueError('Evidence and calibration resolutions differ')
    if evidence.get('user_confirmed') is not True:
        raise ValueError('Manual field checks require user confirmation')
    longitudinal = evidence.get('longitudinal_checks',[])
    lateral = evidence.get('point_checks',[])
    if len(longitudinal) < 3 or not lateral:
        raise ValueError('Need three longitudinal checks and an independent lateral check')
    real = np.asarray([c['measured_m'] for c in longitudinal],dtype=float)
    observed = np.asarray([c['displayed_m'] for c in longitudinal],dtype=float)
    if real.ndim != 1 or observed.shape != real.shape or not np.isfinite([real,observed]).all():
        raise ValueError('Invalid longitudinal evidence')
    if np.ptp(real) < .10 or (real < 0).any() or np.abs(real-observed).max() > max_error:
        raise ValueError('Longitudinal coverage or error failed')
    point_real = np.asarray([c['measured_front_xy_m'] for c in lateral],dtype=float)
    point_observed = np.asarray([c['displayed_front_xy_m'] for c in lateral],dtype=float)
    if (point_real.shape != (len(lateral),2) or point_observed.shape != point_real.shape or
            not np.isfinite([point_real,point_observed]).all()):
        raise ValueError('Invalid lateral evidence')
    point_errors = np.linalg.norm(point_real-point_observed,axis=1)
    if np.abs(point_real[:,1]).max() < .08 or point_errors.max() > max_error:
        raise ValueError('Lateral coverage or error failed')
    H = np.asarray(preview['H'],dtype=float)
    translation = np.array([[1.,0.,offset_x],[0.,1.,offset_y],[0.,0.,1.]])
    result = dict(preview)
    result.update(H=(translation @ H).tolist(),frame_id='base_link',validated=True,
        front_offset_base_link_m=[offset_x,offset_y],
        metres=(np.asarray(preview['metres'])+[offset_x,offset_y]).tolist(),
        validation=dict(method='user_confirmed_field_screen_checks',evidence=evidence,
            longitudinal_errors_m=(observed-real).tolist(),point_errors_m=point_errors.tolist(),
            max_error_m=max_error,scope='initial_static_nav2_commissioning',
            limitations=['Screen readings, not raw pixel reprojection residuals.',
                         'Only right-side independent lateral check available.',
                         'No temporal stability or motion performance certification.',
                         'Fixed 640x480 mount and original calibrated polygon only.']))
    return result
