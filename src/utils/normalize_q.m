function normalized_q = normalize_q(q)
n = norm(q);
if n > 1e-12
    normalized_q = q / n;
else
    normalized_q = [1;0;0;0];
end
end