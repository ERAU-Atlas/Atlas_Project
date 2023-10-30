function [q] = qrotz(angle)

q = [cos(angle/2); 0; 0; sin(angle/2)];

end