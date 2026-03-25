for j = 1:M
    if any(vecnorm(traj(:,1:2)-obs(j,1:2),2,2) <= Reff(j))
        safe = false;
        j
        break;
    end
end