function qdot = calc_qdot(q, omega)

q = normalize(q);
omega_matrix = [
    0 -omega(1) -omega(2) -omega(3); 
    omega(1) 0 omega(3) -omega(2); 
    omega(2) -omega(3) 0 omega(1);
    omega(3) omega(2) -omega(1) 0];

qdot = 0.5*omega_matrix*q;
end