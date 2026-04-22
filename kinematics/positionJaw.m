function success = positionJaw(position)
    success = moveIKReal(arb, position, 1);
end