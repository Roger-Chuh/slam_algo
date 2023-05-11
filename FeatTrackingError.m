
featId2check;
errVecList = [];
for nk = 1 : size(KeyProbZ,1)
ids = inlierId(featId2check);fr = nk; aas = find(KeyProbZ{fr,2} == ids);
errVecList(nk,1) = KeyProbZ{fr,8}(aas);
end
figure(98990);subplot(1,2,1);cla;plot(errVecList);