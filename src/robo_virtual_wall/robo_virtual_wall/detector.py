"""Algorithm extracted from ~/Downloads/detecta_parede_virtual.py.

HSV and morphology are preserved. Wider chevrons additionally use repeated
red/white runs along the band. The Downloads original remains untouched.
"""
import cv2
import numpy as np

ROI_TOPO = 0.10
KERNEL_FECHAMENTO = 25
AREA_MINIMA = 400
ALONGAMENTO_MIN = 2.0
LISTRAS_MIN = 5
FRACAO_BRANCO_MIN = 0.15
MOSTRAR_DESCARTADOS = True

def mascara_cor(hsv, c):
    s_v_min = (c["s_min"], c["v_min"])
    s_v_max = (c["s_max"], c["v_max"])
    if c["h_min"] <= c["h_max"]:
        return cv2.inRange(hsv, (c["h_min"], *s_v_min), (c["h_max"], *s_v_max))
    m1 = cv2.inRange(hsv, (c["h_min"], *s_v_min), (179, *s_v_max))
    m2 = cv2.inRange(hsv, (0, *s_v_min), (c["h_max"], *s_v_max))
    return cv2.bitwise_or(m1, m2)


def mascara_fita(frame, cfg):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    m = mascara_cor(hsv, cfg)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m[: int(m.shape[0] * ROI_TOPO), :] = 0
    return m, hsv


def unir_tracejado(mask):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNEL_FECHAMENTO, KERNEL_FECHAMENTO))
    united = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    # A circular kernel misses gaps along thin distant tape: increasing its
    # diameter also joins neighbouring bands. Directional kernels bridge along
    # the tape while keeping their transverse footprint narrow.
    for angle in (0, 45, 90, 135):
        kernel = np.zeros((43, 43), np.uint8)
        axis = 20*np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])
        start = tuple(np.rint(21-axis).astype(int))
        end = tuple(np.rint(21+axis).astype(int))
        cv2.line(kernel, start, end, 1, 3)
        united = cv2.bitwise_or(united, cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel))
    return united


def repeticoes_na_faixa(contorno, vermelho, branco, origem=(0, 0), forma=None):
    """Count repeated red AND white runs, even when chevron arms touch.

    Sample seven longitudinal lines in the rotated rectangle. Require evidence
    on at least two lines, so a single noisy pixel row cannot validate a band.
    vermelho/branco may be a crop starting at ``origem`` of an image with
    shape ``forma``; sampling stays in full-image coordinates (exact result).
    """
    forma = vermelho.shape if forma is None else forma
    x0, y0 = origem
    (cx, cy), (width, height), angle = cv2.minAreaRect(contorno)
    if width < height:
        width, height = height, width
        angle += 90
    theta = np.deg2rad(angle)
    axis = np.array([np.cos(theta), np.sin(theta)])
    normal = np.array([-axis[1], axis[0]])
    t = np.linspace(-width * .48, width * .48, max(2, int(width)))
    valid = 0
    region = np.zeros_like(vermelho)
    cv2.drawContours(region, [contorno], -1, 255, -1, offset=(-x0, -y0))
    for offset in np.linspace(-height * .3, height * .3, 7):
        points = np.array([cx, cy]) + t[:, None] * axis + offset * normal
        x = np.clip(np.rint(points[:, 0]).astype(int), 0, forma[1] - 1) - x0
        y = np.clip(np.rint(points[:, 1]).astype(int), 0, forma[0] - 1) - y0
        x = np.clip(x, 0, vermelho.shape[1] - 1)
        y = np.clip(y, 0, vermelho.shape[0] - 1)
        labels = np.where(vermelho[y, x] > 0, 1, np.where(branco[y, x] > 0, 2, 0))
        labels[region[y, x] == 0] = 0
        edges = np.r_[0, np.flatnonzero(labels[1:] != labels[:-1]) + 1, len(labels)]
        # Ignore one-pixel transitions; unknown/dark gaps cannot count as white.
        runs = [int(labels[a]) for a, b in zip(edges[:-1], edges[1:]) if b - a >= 2]
        alternating = []
        for value in runs:
            if value == 0:
                continue
            if not alternating or alternating[-1] != value:
                alternating.append(value)
        if alternating.count(1) >= LISTRAS_MIN and alternating.count(2) >= LISTRAS_MIN - 1:
            valid += 1
    return valid >= 2


def parece_fita(contorno, mask_vermelho, hsv):
    """Testa formato + padrão zebrado. Retorna (ok, motivo)."""
    (_, _), (w, h), _ = cv2.minAreaRect(contorno)
    if min(w, h) < 1:
        return False, "fino"
    along = max(w, h) / min(w, h)
    if along < ALONGAMENTO_MIN:
        return False, f"along {along:.1f}"

    # DETECTOR_RECORTE_20261121: work on a crop around the contour and its
    # rotated rectangle (plus margin) instead of the whole 640x480 image.
    # Everything outside the contour was already zeroed, so counts are equal.
    corners = np.vstack((contorno.reshape(-1, 2).astype(float),
                         cv2.boxPoints(cv2.minAreaRect(contorno))))
    x0, y0 = (int(v) for v in np.maximum(np.floor(corners.min(axis=0)).astype(int) - 2, 0))
    x1 = min(mask_vermelho.shape[1], int(np.ceil(corners[:, 0].max())) + 3)
    y1 = min(mask_vermelho.shape[0], int(np.ceil(corners[:, 1].max())) + 3)
    if x1 <= x0 or y1 <= y0:
        return False, "branco 0.00"  # same as the empty full-image region
    forma = mask_vermelho.shape
    mask_vermelho = mask_vermelho[y0:y1, x0:x1]
    hsv = hsv[y0:y1, x0:x1]
    regiao = np.zeros(mask_vermelho.shape, np.uint8)
    cv2.drawContours(regiao, [contorno], -1, 255, -1, offset=(-x0, -y0))

    listras = cv2.bitwise_and(mask_vermelho, regiao)
    n, _, stats, _ = cv2.connectedComponentsWithStats(listras)
    n_listras = sum(1 for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] > 15)

    # pixels claros e pouco saturados (branco) dentro da faixa, fora das listras
    branco = cv2.inRange(hsv, (0, 0, 150), (179, 70, 255))
    branco = cv2.bitwise_and(branco, cv2.bitwise_and(regiao, cv2.bitwise_not(mask_vermelho)))
    frac = cv2.countNonZero(branco) / max(cv2.countNonZero(regiao), 1)
    if frac < FRACAO_BRANCO_MIN:
        return False, f"branco {frac:.2f}"
    if n_listras < LISTRAS_MIN and not repeticoes_na_faixa(contorno, listras, branco,
                                                           (x0, y0), forma):
        return False, f"padrao {n_listras}"
    return True, "ok"


def detectar_paredes(mask_unida, mask_vermelho, hsv, frame=None):
    paredes = []
    contornos, _ = cv2.findContours(mask_unida, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for c in contornos:
        if cv2.contourArea(c) < AREA_MINIMA:
            continue
        ok, motivo = parece_fita(c, mask_vermelho, hsv)
        (_, _), (width, height), _ = cv2.minAreaRect(c)
        # A validated thin straight band already has a well-defined centre.
        # Running Hough on its chevrons creates competing angled edge axes.
        thin_straight = min(width,height) > 0 and max(width,height)/min(width,height) >= 6 and min(width,height) <= 35
        if ok and thin_straight:
            paredes.append(eixo_central_faixa(c))
            continue
        segments = segmentos_do_grupo(c, mask_unida, mask_vermelho, hsv)
        # Several Hough corridors can describe fragments of the same tape.
        # Prefer the full validated contour only when all proposals agree with
        # its axis; preserve distinct branches and separate parallel bands.
        if len(segments) >= 2 and not (ok and mesmo_eixo_do_contorno(c, segments)):
            paredes.extend(segments)
            continue
        if not ok:
            if segments:
                paredes.extend(segments)
                continue
            if frame is not None and MOSTRAR_DESCARTADOS:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (128, 128, 128), 1)
                cv2.putText(frame, motivo, (x, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
            continue
        paredes.append(eixo_central_faixa(c))
    return unir_segmentos_alinhados(consolidar_segmentos(paredes))


def consolidar_segmentos(segmentos):
    """Suppress overlapping hypotheses of one band, keeping longest support.

    Both endpoints of the shorter segment must lie near the longer axis and
    overlap its longitudinal extent. Perpendicular branches stay independent.
    """
    ordered = sorted(segmentos, key=lambda s: -np.linalg.norm(np.subtract(s[1], s[0])))
    kept = []
    for segment in ordered:
        points = np.asarray(segment[:2], dtype=float)
        axis = points[1] - points[0]
        length = np.linalg.norm(axis)
        if length < 1:
            continue
        duplicate = False
        for old in kept:
            origin = np.asarray(old[0], dtype=float)
            old_axis = np.subtract(old[1], origin)
            old_length = np.linalg.norm(old_axis)
            old_axis /= old_length
            if abs(np.dot(axis / length, old_axis)) < np.cos(np.deg2rad(20)):
                continue
            offsets = points - origin
            across = np.abs(offsets @ np.array([-old_axis[1], old_axis[0]]))
            along = offsets @ old_axis
            overlap = max(0., min(along.max(), old_length) - max(along.min(), 0.))
            if across.max() <= 25 and overlap >= 0.6 * length:
                duplicate = True
                break
        if not duplicate:
            kept.append(segment)
    return kept




def segmentos_do_grupo(contorno, mask_unida, mask_vermelho, hsv):
    """Recover distinct band directions inside a merged morphological contour.

    Hough only proposes axes; each narrow support corridor must still pass the
    red/white pattern checks. Segments stay within observed pixel support.
    """
    x, y, width, height = cv2.boundingRect(contorno)
    region = np.zeros_like(mask_unida)
    cv2.drawContours(region, [contorno], -1, 255, -1)
    group_mask = cv2.bitwise_and(mask_unida, region)
    group_points = cv2.findNonZero(group_mask)[:,0,:].astype(float)
    local = group_mask[y:y+height, x:x+width]
    edges = cv2.Canny(local, 50, 150)
    proposals = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=25,
                               minLineLength=45, maxLineGap=20)
    if proposals is None:
        return []
    accepted = []
    proposals = sorted(proposals[:, 0], key=lambda p: -np.hypot(p[2]-p[0], p[3]-p[1]))
    for ax, ay, bx, by in proposals[:100]:
        start = np.array([ax+x, ay+y], dtype=float)
        end = np.array([bx+x, by+y], dtype=float)
        axis = end-start
        length = np.linalg.norm(axis)
        axis /= length
        normal = np.array([-axis[1], axis[0]])
        contour_points = contorno.reshape(-1, 2)
        # Hough may return only one side of a junction. Use it as a direction
        # proposal, extending the support corridor across the observed group.
        middle = (start + end) / 2
        extent = (contour_points - middle) @ axis
        start = middle + axis * extent.min()
        end = middle + axis * extent.max()
        length = float(extent.max() - extent.min())
        # Boundary lines from Hough need an inward shift to sample band centres.
        for offset in [0, 8, -8, 16, -16]:
            centre = (start+end)/2 + offset*normal
            half_width = 12
            corners = np.array([centre-axis*length/2-normal*half_width,
                                centre+axis*length/2-normal*half_width,
                                centre+axis*length/2+normal*half_width,
                                centre-axis*length/2+normal*half_width], np.int32)
            corridor = np.zeros_like(mask_unida)
            cv2.fillConvexPoly(corridor, corners, 255)
            support = cv2.bitwise_and(region, corridor)
            red = cv2.bitwise_and(mask_vermelho, support)
            points = cv2.findNonZero(red)
            if points is None or len(points) < 150:
                continue
            # Validate against the proposed straight band, not the entire junction.
            if not parece_fita(corners.reshape(-1,1,2), red, hsv)[0]:
                continue
            along = (points[:,0,:]-centre) @ axis
            across = (points[:,0,:]-centre) @ normal
            # Centre in actual red support, avoiding systematic boundary offsets.
            centre += normal*np.median(across)
            p1 = centre+axis*along.min()
            p2 = centre+axis*along.max()
            refined = refinar_eixo_grupo(p1, p2, group_points)
            if refined is not None:
                p1,p2 = refined
                centre = (p1+p2)/2
                axis = (p2-p1)/np.linalg.norm(p2-p1)
            duplicate = False
            for old1, old2, _ in accepted:
                old_axis = np.array(old2, float)-old1
                old_axis /= np.linalg.norm(old_axis)
                if abs(np.dot(axis, old_axis)) > np.cos(np.deg2rad(12)):
                    distance = abs(np.dot(centre-np.array(old1), [-old_axis[1],old_axis[0]]))
                    if distance < 25:
                        duplicate = True
                        break
            if not duplicate:
                accepted.append((tuple(int(v) for v in p1), tuple(int(v) for v in p2),
                                 float(np.degrees(np.arctan2(axis[1],axis[0])))))
            break
    return accepted


def mesmo_eixo_do_contorno(contorno, segmentos):
    """True only for proposals supported by one common contour axis."""
    vx, vy, x, y = cv2.fitLine(contorno, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    axis = np.array([vx, vy], dtype=float)
    normal = np.array([-vy, vx], dtype=float)
    origin = np.array([x, y], dtype=float)
    for p1, p2, _ in segmentos:
        points = np.asarray([p1, p2], dtype=float)
        direction = points[1]-points[0]
        length = np.linalg.norm(direction)
        if length < 1 or abs(direction @ axis)/length < np.cos(np.deg2rad(12)):
            return False
        if np.max(np.abs((points-origin) @ normal)) > 15:
            return False
    return True


def unir_segmentos_alinhados(segmentos):
    """Join independently validated tape fragments across short pixel gaps.

    A tight angle/lateral gate protects branches and parallel tape. There is
    no extension beyond the outer observed endpoints.
    """
    result = list(segmentos)
    changed = True
    while changed:
        changed = False
        for i, first in enumerate(result):
            p = np.asarray(first[:2], dtype=float)
            axis = p[1]-p[0]
            length = np.linalg.norm(axis)
            if length < 1:
                continue
            axis /= length
            normal = np.array([-axis[1],axis[0]])
            for j in range(i+1,len(result)):
                q = np.asarray(result[j][:2],dtype=float)
                other = q[1]-q[0]
                other_length = np.linalg.norm(other)
                if other_length < 1 or abs(other @ axis)/other_length < np.cos(np.deg2rad(5)):
                    continue
                if np.max(np.abs((q-p[0]) @ normal)) > 8:
                    continue
                along = (q-p[0]) @ axis
                gap = max(0.,along.min()-length,-along.max())
                if gap > 40:
                    continue
                points = np.vstack((p,q))
                order = np.argsort((points-p[0]) @ axis)
                a,b = points[order[0]],points[order[-1]]
                result[i] = (tuple(int(v) for v in a),tuple(int(v) for v in b),
                             float(np.degrees(np.arctan2(*(b-a)[::-1]))))
                result.pop(j)
                changed = True
                break
            if changed:
                break
    return result


def eixo_central_faixa(contorno):
    """Fit section midpoints rather than weighting irregular contour edges.

    Chevron arms produce unequal boundary sampling on each side. Taking the
    midpoint of the two boundaries in longitudinal bins reduces this bias.
    Endpoints remain within the observed contour's longitudinal extent.
    """
    (cx,cy),(width,height),angle = cv2.minAreaRect(contorno)
    if width < height:
        width,height = height,width
        angle += 90
    theta = np.deg2rad(angle)
    axis = np.array([np.cos(theta),np.sin(theta)])
    normal = np.array([-axis[1],axis[0]])
    origin = np.array([cx,cy])
    points = contorno.reshape(-1,2).astype(float)
    along = (points-origin) @ axis
    across = (points-origin) @ normal
    edges = np.linspace(along.min(),along.max()+1e-6,max(7,min(25,int(width/12)+1)))
    centres = []
    for lo,hi in zip(edges[:-1],edges[1:]):
        selected = (along >= lo) & (along < hi)
        if selected.sum() >= 2:
            left,right = np.percentile(across[selected],[2,98])
            centres.append(origin+axis*np.median(along[selected])+normal*(left+right)/2)
    support = np.asarray(centres,dtype=np.float32) if len(centres) >= 4 else points.astype(np.float32)
    vx,vy,x0,y0 = cv2.fitLine(support,cv2.DIST_HUBER,0,.01,.01).flatten()
    direction,centre = np.array([vx,vy]),np.array([x0,y0])
    extent = (points-centre) @ direction
    start,end = centre+direction*extent.min(),centre+direction*extent.max()
    return (tuple(int(round(v)) for v in start),tuple(int(round(v)) for v in end),
            float(np.degrees(np.arctan2(vy,vx))))


def refinar_eixo_grupo(start, end, points):
    """Centre an accepted branch using both band edges away from junctions.

    Hough supplies orientation hypotheses, not the final tape centre. Wide
    cross-sections or sections truncated by the search corridor are rejected,
    preventing a crossing branch from dragging the fitted centreline sideways.
    """
    axis = end-start
    length = np.linalg.norm(axis)
    if length < 45:
        return None
    axis /= length
    normal = np.array([-axis[1],axis[0]])
    origin = (start+end)/2
    along = (points-origin) @ axis
    across = (points-origin) @ normal
    near = np.abs(across) <= 35
    if near.sum() < 100:
        return None
    t,n = along[near],across[near]
    edges = np.linspace(t.min(),t.max()+1e-6,max(7,min(25,int(length/12)+1)))
    centres,widths = [],[]
    for lo,hi in zip(edges[:-1],edges[1:]):
        selected = (t >= lo) & (t < hi)
        if selected.sum() < 15:
            continue
        left,right = np.percentile(n[selected],[2,98])
        if left < -32 or right > 32 or right-left > 40 or right-left < 2:
            continue
        centres.append(origin+axis*np.median(t[selected])+normal*(left+right)/2)
        widths.append(right-left)
    if len(centres) < 4:
        return None
    centres = np.asarray(centres,dtype=np.float32)
    if np.ptp((centres-origin) @ axis) < 40:
        return None
    vx,vy,cx,cy = cv2.fitLine(centres,cv2.DIST_HUBER,0,.01,.01).flatten()
    fitted_axis = np.array([vx,vy],dtype=float)
    if abs(fitted_axis @ axis) < np.cos(np.deg2rad(12)):
        return None
    fitted_normal = np.array([-vy,vx],dtype=float)
    centre = np.array([cx,cy],dtype=float)
    support = np.abs((points-centre) @ fitted_normal) <= np.median(widths)*.6+2
    extent = (points[support]-centre) @ fitted_axis
    if not len(extent):
        return None
    return centre+fitted_axis*extent.min(),centre+fitted_axis*extent.max()
