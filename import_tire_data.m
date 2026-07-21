function [out] = import_tire_data(filepath)
% import_tire_data loads the data from a tire testing run
%   I use this to remove irrelevant variables
%	and to output a struct containing the data
%   so that data from multiple runs can be in the same workspace with no name conflicts
%
%   inputs: filepath
%
%   outputs: a struct "out", with the fields:
%	AMBTMP, channel, ET, FX, FY, FZ, IA, MX, MZ, N, ...
%   NFX, NFY, P, RE, RL, RST, RUN, SA, SL, source, ...
%   SR, testid, tireid, TSTC, TSTI, TSTO, V
%
%   commenting lines out of this function to ignore some variables could be helpful

    load(filepath);

    % optional matter of sign convention
    FZ = abs(FZ);
    
    % data is taken on sandpaper. To estimate on concrete, reduce by 2/3
    irl_factor = .6666;

    % variable names are defined by tire data. Change them here if you wish
    % load into struct
    out.AMBTMP = AMBTMP;
    %out.channel = channel;			%channel is not in round 5
    out.ET = ET;
    out.FX = FX .* irl_factor;
    out.FY = FY .* irl_factor;
    out.FZ = FZ;
    out.IA = IA;
    %out.MX = MX;
    %out.MZ = MZ;
    %out.N = N;
    out.NFX = NFX .* irl_factor;
    out.NFY = NFY .* irl_factor;
    out.P = P;
    %out.RE = RE;
    %out.RL = RL;
    %out.RST = RST;
    %out.RUN = RUN;			% RUN is not in round 5
    out.SA = SA;
    out.SL = SL;
    %out.source = source;
    out.SR = SR;
    out.testid = testid;
    out.tireid = tireid;
    %out.TSTC = TSTC;
    %out.TSTI = TSTI;
    %out.TSTO = TSTO;
    %out.V = V;
end

