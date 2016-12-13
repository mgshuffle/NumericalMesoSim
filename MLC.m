function p = MLC(s,thisLIdx)
global laneList
len = sum(laneList(thisLIdx,2:3));
    buff = 20;%m
    if len > buff
        p = (min(s,len-buff))/(len-buff);
    else
        p = 1;
    end
end