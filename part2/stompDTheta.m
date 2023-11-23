function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);
% Variable declaration
dtheta = zeros(nJoints, nDiscretize);

% Iterate over all joints
for i = 1:nJoints
    temp = em{i}';
    for k = 1:nDiscretize
        dtheta(i,k) = temp(k,:)*trajProb(:,k);
    end

end
