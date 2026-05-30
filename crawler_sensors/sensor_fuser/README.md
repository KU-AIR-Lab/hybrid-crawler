# sensor_fuser

Combines raw left/right sensor arrays into unified control feedback:

- Cable 10 contact sensors -> `/cable/Fc_act`
- Front soft [left,right] contact -> `/soft/front/Fc_act`
- Rear soft [left,right] contact -> `/soft/rear/Fc_act`
- Front soft [left,right] pressure -> `/soft/front/p_act`
- Rear soft [left,right] pressure -> `/soft/rear/p_act`
