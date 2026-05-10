function [T, Ts] = DH(obj, config)
    if nargin < 2
        config = obj.jointAngles(1:4);
    end

    % Pull DH parameters from robot structure
    % Adjust these field names to match however you store them in robotStructure

    theta = config;

    % Original DH logic
    if isa(theta, 'sym')
        T  = sym(eye(4));
        Ts = cell(length(theta), 1);
    else
        T  = eye(4);
        Ts = cell(length(theta), 1);
    end

    for i = 1:length(theta)
        ct = cos(theta(i));
        ca = cos(obj.alpha(i));
        st = sin(theta(i));
        sa = sin(obj.alpha(i));

        T_ = [ct  -st*ca   st*sa   obj.a(i)*ct;
              st   ct*ca  -ct*sa   obj.a(i)*st;
               0      sa      ca      obj.d(i);
               0       0       0         1];

        T      = T * T_;
        Ts{i}  = T;        % each cell is the cumulative transform to joint i
    end
end
