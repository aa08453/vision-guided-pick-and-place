function moveByJoints(obj, jointAnglesCommanded)
    arb.setpos(4,jointAnglesCommanded(4),speed);
    arb.setpos(3,jointAnglesCommanded(3),speed);
    arb.setpos(2,jointAnglesCommanded(2),speed);
    arb.setpos(1,jointAnglesCommanded(1),speed);
end
