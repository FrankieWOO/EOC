u = 0:0.05:1;

for i = 1:length(u)
    if u(i) <= 0.5
        D1(i) = u(i)/0.5;
        D2(i) = 0;
    else
        D1(i) = 1;
        D2(i) = ( u(i)-0.5)/0.5;
    end
end

figure
plot(u,D1,u,D2)

%%
