import numpy as np

def vec_cos(vec1, vec2):
    return np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))

def is_line(vec_list, threshold = 0.8,):
    if vec_list is not None:
        for i in range(len(vec_list) - 1):
            if vec_cos(vec1=vec_list[i], vec2=vec_list[i+1]) < threshold:
                return False
        return True 
    raise NotImplementedError

def gesture_recognition(hand_landmarks):
    lm = hand_landmarks.landmark  # Assuming hand_landmarks is the list of landmarks
    thr = 0.1
    xy_ratio_thr = 1.2
    yx_ratio_thr = 2
    eps = 1e-3
    cos_thr = 0.8

    feature_pt = np.array([(lm[8].x + lm[12].x) / 2, (lm[8].y + lm[12].y) / 2])
    wrist_pt = np.array([lm[0].x, lm[0].y])
    feature_vec = feature_pt - wrist_pt
    xy_feature_ratio = np.abs(feature_vec[0]) / (np.abs(feature_vec[1]) + eps)
    yx_feature_ratio = np.abs(feature_vec[1]) / (np.abs(feature_vec[0]) + eps)
    
    # Calculate vectors
    # vec01 = np.array([lm[1].x - lm[0].x, lm[1].y - lm[0].y])

    # vec02 = np.array([lm[2].x - lm[0].x, lm[2].y - lm[0].y])
    vec23 = np.array([lm[3].x - lm[0].x, lm[3].y - lm[0].y])
    vec34 = np.array([lm[4].x - lm[0].x, lm[4].y - lm[0].y])
    # thumb = [vec01, vec02, vec03, vec04]
    # thumb = [vec02, vec23, vec34]
    thumb = [vec23, vec34]

    # vec05 = np.array([lm[5].x - lm[0].x, lm[5].y - lm[0].y])
    vec56 = np.array([lm[6].x - lm[5].x, lm[6].y - lm[5].y])
    vec68 = np.array([lm[8].x - lm[6].x, lm[8].y - lm[6].y])
    # pointer = [vec05, vec56, vec68]
    pointer = [vec56, vec68]

    # vec09 = np.array([lm[9].x - lm[0].x, lm[9].y - lm[0].y])
    vec9_10 = np.array([lm[10].x - lm[9].x, lm[10].y - lm[9].y])
    vec10_12 = np.array([lm[12].x - lm[10].x, lm[12].y - lm[10].y])

    # middle = [vec09, vec9_10, vec10_12]
    middle = [vec9_10, vec10_12]

    # vec0_13 = np.array([lm[13].x - lm[0].x, lm[13].y - lm[0].y])
    vec13_14 = np.array([lm[14].x - lm[13].x, lm[14].y - lm[13].y])
    vec14_16 = np.array([lm[16].x - lm[14].x, lm[16].y - lm[14].y])

    # ring = [vec0_13, vec13_14, vec14_16]
    ring = [vec13_14, vec14_16]
    
    # vec0_17 = np.array([lm[17].x - lm[0].x, lm[17].y - lm[0].y])
    vec17_18 = np.array([lm[18].x - lm[17].x, lm[18].y - lm[17].y])
    vec18_20 = np.array([lm[20].x - lm[18].x, lm[20].y - lm[18].y])

    # pinky = [vec0_17, vec17_18, vec18_20]
    pinky = [vec17_18, vec18_20]
    
    thumb_is_line = is_line(thumb, cos_thr)
    pointer_is_line = is_line(pointer, cos_thr)
    middle_is_line = is_line(middle, cos_thr)
    ring_is_line = is_line(ring, cos_thr)
    pinky_is_line = is_line(pinky, cos_thr)

    if ring_is_line and pinky_is_line:
        if thumb_is_line and pointer_is_line and middle_is_line:
            return "forward"
    else:
        if pointer_is_line and middle_is_line:
            if xy_feature_ratio > xy_ratio_thr: 
                # left or right
                return "left" if feature_vec[0] > 0 else "right"
            if yx_feature_ratio > yx_ratio_thr:
                # up or down 
                return "up" if feature_vec[1] < 0 else "down"

        elif not pointer_is_line and not middle_is_line:
            return "backward"
    return "control"