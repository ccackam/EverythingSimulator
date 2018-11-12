function output = cascade(controllers,x,r)
    commands(1,1) = r(1);
    for i = 1:length(controllers)
        commands(i+1,1) = controllers(i).master(x,commands(end));
    end
    u = commands(end);
    r = commands(1:end-1);
    output = {u,r};
    % Note, this must be changed for MISO and MIMO
end