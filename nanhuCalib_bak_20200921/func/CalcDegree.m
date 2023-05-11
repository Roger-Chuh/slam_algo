function theta = CalcDegree(vec1, vec2)

theta = min([rad2deg(acos(dot(vec1, vec2)/norm(vec1)/norm(vec2))); rad2deg(acos(dot(vec1, -vec2)/norm(vec1)/norm(vec2)))]);

end