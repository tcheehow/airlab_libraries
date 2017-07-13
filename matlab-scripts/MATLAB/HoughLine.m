function out = HoughLine(acc, thetas, rhos)
    candidate = max(acc(:));
%     [I_row, I_column] = ind2sub(size(acc), find(acc==candidate));
    [I_row, I_column] = ind2sub(size(acc), find(acc>=candidate));
    r = rhos(I_row);
    t = thetas(I_column);
    out = [r', rad2deg(t')];
    out = sort(out, 1);
end