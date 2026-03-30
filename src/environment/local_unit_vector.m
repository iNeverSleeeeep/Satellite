function vecHat = local_unit_vector(vec)
%LOCAL_UNIT_VECTOR 对三维向量归一化，接近零向量时返回零向量。
%
%   计算公式：
%   vecHat = vec / |vec|
%
%   当 |vec| 接近 0 时，为避免数值发散，直接返回零向量。

vecNorm = norm(vec);
if vecNorm < eps
    vecHat = zeros(size(vec));
else
    vecHat = vec / vecNorm;
end
end
