 function [categoryClassifier] = train_classifier()
    imds = imageDatastore('dataset','IncludeSubfolders',true,'LabelSource','foldernames');
    tbl = countEachLabel(imds);
    figure
    montage(imds.Files(1:16:end))
    [trainingSet, validationSet] = splitEachLabel(imds, 0.8, 'randomize');
    bag = bagOfFeatures(trainingSet, 'PointSelection','Detector');
    categoryClassifier = trainImageCategoryClassifier(trainingSet, bag);
    confMatrix = evaluate(categoryClassifier, validationSet);
    disp('average accuracy :'); % 100% accuracy on test data OMG :o 
    mean(diag(confMatrix))
end