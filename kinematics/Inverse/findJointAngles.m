function [theta1, theta2, theta3, theta4] = findJointAngles(x,y,z, phi)
    d1 = 13.7; a2 = 10.5; a3 = 10.5; a4 = 11;
    theta1.a = atan2(y, x);
    theta1.b = atan2(-y, -x);

    x_bar = x/cos(theta1.a) - a4 * cos(phi);
    y_bar = y/sin(theta1.a) - a4 * cos(phi);
    z_bar = z - d1 - a4 * sin(phi);

    u = (x_bar^2 + z_bar^2 -a2^2 - a3^2) / (2 * a2 * a3);

    theta3.a = atan2(sqrt(1-u^2), u);
    theta3.b = atan2(-sqrt(1-u^2), u);

   
    denom = (a2^2 + 2 * a2 * a3 * u + a3^2);
    costh2.a = y_bar * (a2 + a3 * u) + z_bar * (a3 * sqrt(1 - u^2));
    sinth2.a = z_bar * (a2 + a3 * u) - y_bar * (a3 * sqrt(1 - u^2));

    costh2.b = y_bar * (a2 + a3 * u) - z_bar * (a3 * sqrt(1 - u^2));
    sinth2.b = z_bar * (a2 + a3 * u) + y_bar * (a3 * sqrt(1 - u^2));

    costh2.a = costh2.a/denom;
    sinth2.a = sinth2.a/denom;

    costh2.b = costh2.b/denom;
    sinth2.b = sinth2.b/denom;


    theta2.a = atan2(sinth2.a, costh2.a);
    theta2.b = atan2(sinth2.b, costh2.b);

    theta4.a = phi - theta3.a - theta2.a;
    theta4.b = phi - theta3.b - theta2.b;



end 