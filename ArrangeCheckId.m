if ~isempty(obj.feat2check)
    
    checkId = find(ismember(inlierId, obj.feat2check));
    checkId131 = checkId;
    toCheck = [obj.feat2check];
    [is,ia] = sort(toCheck);
    [ia1,ia2] = sort(ia);
    checkId = checkId131(ia2);
    [~,sfd] = sort(checkId);
    [~,sfdd] = sort(toCheck);
    sakhg = sfd - sfdd;
     
end