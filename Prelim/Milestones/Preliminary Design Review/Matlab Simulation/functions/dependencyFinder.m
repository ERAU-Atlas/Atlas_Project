function [] = dependencyFinder(functionName)
% prints dependencies of given function

[fList] = matlab.codetools.requiredFilesAndProducts('myFun.m');

flist{:}

end