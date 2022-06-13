Controllers = ["SMC", "PID"];
x={}
for i = 1:length(Controllers)
    Controller = Controllers(i);
    x{end+1}=i
end