save_path = fullfile('..', 'simulation_tests');

var = linspace(0, 0.01, 6);
drift_rate = 0:0.01:0.05;
outliers = 0:2:10;

mixture.var = 0.005;
mixture.outliers = 5;
mixture.drift_rate = 0.025;

dg = DataGenerator();

for j = 1:50
    dg.generateRandomTrajectory('flatness', 1e-1, 'numPoints', 100);
    
    for i = 1:length(var)
        path = fullfile(save_path, sprintf('noise/noise_%f/run_%d', var(i), j));
        noise_var = [var(i) * ones(1,3), 2*var(i) * ones(1,3)];
        dg.generateData('NoiseVar', noise_var);
        dg.save(path);
    end
    
    for i = 1:length(drift_rate)
        path = fullfile(save_path, sprintf('drift/drift_%f/run_%d', drift_rate(i), j));
        dg.generateData('Drift', drift_rate(i));
        dg.save(path);
    end

    for i = 1:length(outliers)
        path = fullfile(save_path, sprintf('outliers/outliers_%d/run_%d', outliers(i), j));
        dg.generateData('Outliers', outliers(i)/100);
        dg.save(path);
    end
    
    path = fullfile(save_path, sprintf('mixture/run_%d', j));
    noise_var = [mixture.var * ones(1,3), 2*mixture.var * ones(1,3)];
    dg.generateData('NoiseVar', noise_var, ...
                    'Outliers', mixture.outliers/100, ...
                    'Drift', mixture.drift_rate);
    dg.save(path);
end
