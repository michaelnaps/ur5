function [q, success] = inverseKinematics_ur5(q0, S, M, Tgoal, eomg, ev)
q = q0;

Tsb = FKinSpace(M, S, q);
Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * Tgoal));

maxIter = 100;
count = 0;
err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
while err && count < maxIter
    q = q + pinv(JacobianSpace(S, q)) * Vs;
    count = count + 1;
    Tsb = FKinSpace(M, S, q);
    Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * Tgoal));
    err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
end

% if max number of iterations reached do not attempt modification
if err
    success = ~err;
    return;
end

% modification loop: place q values in range of -pi to pi
% err_range = (q > -pi) & (q < pi);
% for i = 1:length(q)
%     while ~err_range(i)
%         if (q(i) < -pi)
%             q(i) = q(i) + 2*pi;
%         elseif (q(i) > pi)
%             q(i) = q(i) - 2*pi;
%         end
% 
%         err_range(i) = (q(i) > -pi) && (q(i) < pi);
%     end
% end
% 
% % check the range and return
% if sum(~err_range) > 0
%     err = 1;
% end

success = ~ err;
end