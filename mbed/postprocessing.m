% this function is called from stm32F4xx_make_rtw_hook (exit-hook)
% ==================================================================
function postprocessing(modelName)
BuildApplication = get_param(modelName,'BuildApplication');
DownloadApplication = get_param(modelName,'DownloadApplication');
postprocessingMbed(modelName, strcmp(BuildApplication, 'on'), strcmp(DownloadApplication, 'on'));
end
