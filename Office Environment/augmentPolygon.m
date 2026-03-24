


function Pol_ = augmentPolygon(Pol, n)

for j = 1:size(Pol,1)
pol = Pol{j};
nEdges = size(pol, 1);
pol_ = [];
for i = 1 : nEdges-1
   
    x = linspace(pol(i,1), pol(i+1,1), n);
    y = linspace(pol(i,2), pol(i+1,2), n);
    if i>1 
        pol_ = [pol_; x(2:end)',y(2:end)'];
    else
        pol_ = [pol_; x',y'];
    end
end
Pol_{j} = pol_;
end
end