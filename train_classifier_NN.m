function net = train_classifier_NN()
    imds = imageDatastore('dataset','IncludeSubfolders',true,'LabelSource','foldernames');
    labelCount = countEachLabel(imds);
    % compute the size of the image
    img = readimage(imds, 1);
    sizeImage = size(img)
    % specify training and validation sets
    [imdsTrain, imdsValidation] = splitEachLabel(imds,0.8,'randomize');
    % Define Network Architecture
    layers = [
        imageInputLayer([sizeImage(1), sizeImage(2), 3])
        
        convolution2dLayer(40,25,'Padding','same')
        batchNormalizationLayer
        reluLayer
        
        maxPooling2dLayer(26,'Stride',2)
        
        convolution2dLayer(40,50,'Padding','same')
        batchNormalizationLayer
        reluLayer
        
        maxPooling2dLayer(26,'Stride',2)
        
        convolution2dLayer(40,100,'Padding','same')
        batchNormalizationLayer
        reluLayer
        
        maxPooling2dLayer(26,'Stride',2)
        
        convolution2dLayer(40,200,'Padding','same')
        batchNormalizationLayer
        reluLayer
        
        fullyConnectedLayer(5)
        softmaxLayer
        classificationLayer];
    
    % specify Training Options
    options = trainingOptions('sgdm', ...
                            'InitialLearnRate',0.01, ...
                            'MaxEpochs',30, ...
                            'Shuffle','every-epoch', ...
                            'ValidationData',imdsValidation, ...
                            'ValidationFrequency',30, ...
                            'Verbose',false, ...
                            'Plots','training-progress',...
                            'ExecutionEnvironment','gpu',...
                            'MiniBatchSize',16);
                       
    net = trainNetwork(imdsTrain,layers,options);
    
    % Classify Validation Images and Compute Accuracy
    
    YPred = classify(net,imdsValidation);
    YValidation = imdsValidation.Labels;

    accuracy = sum(YPred == YValidation)/numel(YValidation)
                        
end