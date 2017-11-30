close

n = size(current_distance)

% sliding median filter
buf_0 = zeros(8,1);
out_0 = [];
count = 1;
for i=1:n
    if (count >= 9)
        count = 1;
    end
    buf_0(count) = current_distance(i);
    count = count + 1;
    out_0 = [out_0; median(buf_0)];
end

% sorted median filter
buf_1 = zeros(8,1);
out_1 = [];
count = 1;
high = true;
for i=1:n
    if (count >= 9)
        count = 1;
    end
    current_distance(i)
    buf_1 = [buf_1; current_distance(i)]; 
    count = count + 1;
    new = (sort(buf_1));
    if (high)
        new = new(1:8);
    else
        new = new(2:9);
    end
    high = ~high;
    buf_1 = new;
    out_1 = [out_1; median(buf_1)];
end

plot(current_distance, 'r-')
hold on
plot(out_0, 'g-')
hold on
plot(out_1, 'b-')
hold off

