function u = cascade(controllers,x,r)
    u = r;
    for i = 1:length(controllers)
        u = controllers(i).master(x,u);
    end
end