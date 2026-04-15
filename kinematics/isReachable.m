function reachable = isReachable(x, y, z, phi)
    d1 = 13.7; a2 = 10.5; a3 = 10.5; a4 = 11;
    r = sqrt(x^2 + y^2);
    s = z - d1;
    r_bar = r - a4 * cos(phi);
    s_bar = s - a4 * sin(phi);
    cos_arg = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);
    reachable = abs(cos_arg) <= 1;
end