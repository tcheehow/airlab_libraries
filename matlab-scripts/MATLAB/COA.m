function [out, centre] = COA(ranges) %radial distance, angle
%% Centre of Mass Finder

    out = [];
    
    for i = 1:(length(ranges)-1)
        sxy_new = find_sxy(ranges(i:i+1, :));
        out = [out; sxy_new];
    end
    
    ss  = out(:,1);
    xs  = out(:,2);
    xs  = (xs .* ss) / sum(ss);
    ys  = out(:,3);
    ys  = (ys .* ss) / sum(ss);
    
    sbar    = sum(ss);
    xbar    = sum(xs);
    ybar    = sum(ys);
    centre  = [xbar, ybar]
    
    function sxy = find_sxy(twin)

    ra      = twin(1,1);
    rb      = twin(2,1);
    aa      = twin(1,2);
    ab      = twin(2,2);
    dtheta  = aa - ab;

    sc      = 0.5 * ra * rb * sin(dtheta);
    xc      = (1/3) * (ra*cos(aa) + rb*cos(ab));
    yc      = (1/3) * (ra*sin(aa) + rb*sin(ab));

    sxy     = [sc, xc, yc];
    end



end

