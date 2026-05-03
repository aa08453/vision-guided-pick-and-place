function moveByJoints(obj, jointAnglesCommanded) %TODO: Set this up to work in simulation too
    orb.arb.setpos(4,jointAnglesCommanded(4),speed);
    orb.arb.setpos(3,jointAnglesCommanded(3),speed);
    orb.arb.setpos(2,jointAnglesCommanded(2),speed);
    orb.arb.setpos(1,jointAnglesCommanded(1),speed);
end
