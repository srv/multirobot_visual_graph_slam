% Normalizes angles to the interval  (-pi,pi]
function angles = normalize(angles)
    angles=angles+(2*pi)*floor((pi-angles)/(2*pi));
