const
  MAZE_SIZE = 16;

type
  TTrajControlMode = (cmManual, cmScript);

  TTimeMeasures = record
    StartTime, intermediate, FinalTime: double;
  end;

  TPose = record
    x, y, theta: double;
  end;


// Global Variables
var
  irobot: integer;
  t: double;
  TimeMeasures: TTimeMeasures;
  go: boolean;
  side: double;

  TrajControlMode: TTrajControlMode;

  UpdateMap: boolean;

  dists: array[0..4] of double;

  state: integer;
  CurTargetIdx: integer;

  v,w : double;

  // Estimated Robot Pose
  x_r, y_r, theta_r, e_theta: double;

//Procedure, functions and Routines

procedure UpdateTimes(var TM: TTimeMeasures);
var x, y: double;
begin
  if RCButtonPressed(24, 2) then begin
    SetRobotPos(0, 0, 0, 0.08, rad(90));
    TM.StartTime := t;
    TM.intermediate := 0;
    TM.FinalTime := 0;
    go := true;
  end else begin
    go := false;
  end;

  x := GetRobotX(0);
  y := GetRobotY(0);

  if (sqrt(sqr(x - 7 * side) +sqr(y - 7 * side)) < 0.05) and (TM.intermediate = 0) then begin
    TM.intermediate := t;
  end;

  if (sqrt(sqr(x) +sqr(y)) < 0.05) and (TM.intermediate <> 0) and (TM.FinalTime = 0)  then begin
    TM.FinalTime := t;
  end;

  SetRCValue(25, 2, format('%3.5g',[t - TM.StartTime]));

  if TM.intermediate <> 0 then begin
    SetRCValue(26, 2, format('%3.5g',[TM.intermediate - TM.StartTime]));
  end else begin
    SetRCValue(26, 2, '');
  end;

  if TM.FinalTime <> 0 then begin
    SetRCValue(27, 2, format('%3.5g',[TM.FinalTime - TM.StartTime]));
  end else begin
    SetRCValue(27, 2, '');
  end;

end;


procedure stop();
begin
  v := 0;
  w := 0;
end;

procedure gotoXY(x, y, angle, xp, yp: double; var v, w: double);
var e_theta, theta_p: double;
begin
  theta_p := atan2(yp - y, xp - x);
  e_theta := DiffAngle(theta_p, angle);
  v := 0.4;
  w := 10 * e_theta;
end;


procedure rotate(angle, target_angle: double; var v, w: double);
var e_theta: double;
begin
  e_theta := DiffAngle(target_angle, angle);
  v := 0;
  w := 10 * e_theta;
end;

procedure ParseUDPMessage(mess: string; var v, w: double);
var Text: TStringList;
    line: integer;
    s: string;
begin
  Text := TStringList.create;
  try
    Text.text := mess;
    SetRCValue(5, 7, inttoStr(Text.count));
    line := 0;
    while line < Text.Count do begin
      s := Text[line];
      if s = 'V' then begin
        inc(line);
        if line >= Text.Count then break;
        v := StrToFloat(Text[line]);
      end else if s = 'W' then begin
        inc(line);
        if line >= Text.Count then break;
        w := StrToFloat(Text[line]);
      end else if s = 'Script' then begin
        TrajControlMode := cmScript;
        SetRCValue(2, 2, 'Script');
       end else if s = 'Manual' then begin
        TrajControlMode := cmManual;
        SetRCValue(2, 2, 'Manual');
        writeln('Manual')
       end else if s = 'map' then begin
        UpdateMap := True
      end;
      inc(line);
    end;
  finally
    Text.free;
  end;
end;

procedure ManualTrajControl();
var txt: String;
    Vmax, Wmax: double;
begin
  txt := ReadUDPData();
  ParseUDPMessage(txt, v, w);
  Vmax := 0.5;
  Wmax := 4;

  V := 0;
  W := 0;

  if KeyPressed(vk_down) then begin
    V := -1;
  end else if KeyPressed(vk_left) then begin
    W := 1;
  end else if KeyPressed(vk_right) then begin
    W := -1
  end else if KeyPressed(vk_up) then begin
    V := 1;
  end;

  V := V * Vmax;
  W := W * Wmax;
end;

procedure ScriptTrajControl();
var txt: String;
    tries: integer;

begin
  // Read UDP data
  txt := '';
  while (txt = '') and (tries < 500) do begin
    inc(tries);

    txt := ReadUDPData();
    if RCButtonPressed(2, 3) then begin
      TrajControlMode := cmManual;
      exit;
    end;
  end;

  SetRCValue(4, 7, txt);
  ParseUDPMessage(txt, v, w);
  //writeln(FloatToStr(v));

{
  V := 0;
  W := 0;

  dp_tresh := 0.05;

  // This will be forbiden in the next challenge!
  x_r := GetRobotX(0);
  y_r := GetRobotY(0);
  theta_r := GetRobotTheta(0);

  SetRCValue(12, 2, format('%3.1g',[x_r]));
  SetRCValue(13, 2, format('%3.1g',[y_r]));
  SetRCValue(14, 2, format('%3.1g',[deg(theta_r)]));

  x_t := BestPath.targets[CurTargetIdx].x;
  y_t := BestPath.targets[CurTargetIdx].y;

  SetRCValue(7, 2, format('%d',[CurTargetIdx]));
  SetRCValue(8, 2, format('%3.1g',[x_t]));
  SetRCValue(9, 2, format('%3.1g',[y_t]));

  if (state = 0) and go then begin
    state := 1;
    go := false;
  end else if (state = 1) and (abs(sqrt(sqr(x_r - x_t) + sqr(y_r - y_t))) < dp_tresh) then begin
    inc(CurTargetIdx);
    if CurTargetIdx > BestPath.numTargets - 1 then begin
      //state := 2;
      state := 0;
      //CurTargetIdx := BestPath.numTargets - 1;
      dec(CurTargetIdx);
    end;
  end else if (state = 2) and (abs(sqrt(sqr(x_r - x_t) + sqr(y_r - y_t))) < dp_tresh) then begin
    dec(CurTargetIdx);
    if CurTargetIdx < 0 then begin
      state := 0;
      CurTargetIdx := 0;
    end;
  end;

  if state = 0 then begin
    stop(V, W);
    //CurTargetIdx := 1;
  end else if state = 1 then begin
    gotoXY(x_r, y_r, theta_r, x_t, y_t, V, W);
  end else if state = 2 then begin
    gotoXY(x_r, y_r, theta_r, x_t, y_t, V, W);
  end;

  SetRCValue(4, 4, format('%d',[state]));
}
end;

procedure VWToV1V2(var v1, v2: double);
var kw, b: double;
begin
  b := 12.5;
  Kw := 10;
  V1 := (V - W/b) * kw;
  V2 := (V + W/b) * kw;
end;


procedure MapFromFile(Fname: string);
var Text: TStringList;
    s: string;
    i, r, c, rgb: integer;
    x, y, z, h: double;
begin
  h := 0.15;
  Text := TStringList.create;
  try
    Text.loadFromFile(Fname);
    if Text.count < MAZE_SIZE then exit;
    ClearObstacles();

    // Surrounding Walls
    z := h/2;
    rgb := $555555;
    for i := 0 to MAZE_SIZE do begin
      x := (i - 1) * side;
      y := (i - 1) * side;
      AddOBstacleBox('Botwall' + IntToStr(i), x, -side, z, side, side, h, rgb);
      AddOBstacleBox('Rightwall' + IntToStr(i), MAZE_SIZE * side, y, z, side, side, h, rgb);
      AddOBstacleBox('topwall' + IntToStr(i), x + side, MAZE_SIZE * side, z, side, side, h, rgb);
      AddOBstacleBox('Lefttwall' + IntToStr(i), -side, y + side, z, side, side, h, rgb);
    end;

    for r := 0 to MAZE_SIZE - 1 do begin
      s := Text[r];
      if length(s) < MAZE_SIZE then exit;
      for c := 0 to MAZE_SIZE - 1 do begin
        y := ((MAZE_SIZE - 1) - r) * side;
        x := c * side;
        z := -1;
        if s[c + 1] = '1' then begin
          z := 0;
          rgb := $0A8CFA;
        end else if s[c + 1] in ['0', '2'] then begin
          z := h/2;
          rgb := $555555;
          rgb := $0A8CFA;
          AddOBstacleBox('wall' + IntToStr(c) + IntToStr((MAZE_SIZE - 1) - r), x, y, z, side, side, h, rgb);
        end;
        //AddOBstacleBox('wall' + IntToStr(c) + IntToStr((MAZE_SIZE - 1) - r), x, y, z, side, side, h, rgb);
      end;
    end;


  finally
    Text.free;
  end;
end;


function BuildMessage(MType, data: string): string;
var Text: TStringList;
begin
  Text := TStringList.create;
  try
    Text.add(MType);
    Text.add(data);
    result := Text.Text;
  finally
    Text.free;
  end;
end;

function BuildSensorMessage(d0, d1, d2, d3, d4: double): string;
var Text: TStringList;
begin
  Text := TStringList.create;
  try
    Text.add('Sensors');
    Text.add(FloatToStr(d0));
    Text.add(FloatToStr(d1));
    Text.add(FloatToStr(d2));
    Text.add(FloatToStr(d3));
    Text.add(FloatToStr(d4));
    result := Text.Text;
  finally
    Text.free;
  end;
end;



procedure Control;
var txt, send_text: string;
    i: integer;
    v1, v2: double;
begin
  t := t + ScriptPeriod();
  UpdateTimes(TimeMeasures);

  // Send robot info via UDP
  send_text := BuildMessage('RX', FloatToStr(GetRobotX(0)));
  send_text :=  send_text + BuildMessage('RY', FloatToStr(GetRobotY(0)));
  send_text :=  send_text + BuildMessage('RY', FloatToStr(deg(GetRobotTheta(0))));
  send_text :=  send_text + BuildMessage('RS', IntToStr(state));
  send_text :=  send_text + BuildMessage('RI', IntToStr(CurTargetIdx));

  SetRCValue(14, 2, format('%3.1g',[deg(GetRobotTheta(0))]));

  for i := 0 to 4 do begin
    dists[i] := GetSensorVal(irobot, i + 1);
    SetRCValue(15, 2 + i, format('%3.1g',[dists[i]]));
  end;

  send_text := send_text + BuildSensorMessage(dists[0], dists[1], dists[2], dists[3], dists[4] );

  WriteUDPData('127.0.0.1', 8421, send_text);

  case TrajControlMode of
    cmManual: ManualTrajControl();
    cmScript: ScriptTrajControl();
  end;



  if RCButtonPressed(11, 3) then begin
    SetRobotPos(0, GetRCValue(12, 3), GetRCValue(13, 3), 0.15, rad(GetRCValue(14, 3)));
  end;

  if RCButtonPressed(2, 3) then begin
    TrajControlMode := cmManual;
  end else if RCButtonPressed(2, 4) then begin
    txt := ReadUDPData();
    while (txt <> '') do begin
      txt := ReadUDPData();
    end;

    TrajControlMode := cmScript;
  end;

  VWToV1V2(v1, v2);
  SetAxisVoltageRef(irobot, 0, v1);
  SetAxisVoltageRef(irobot, 1, v2);

  SetRCValue(5, 4, format('%d',[round(v)]));
  SetRCValue(6, 4, format('%d',[round(w)]));

 // v := sqrt(sqr(GetRobotVx(0)) + sqr(GetRobotVy(0)));
  SetRCValue(5, 2, format('%3.1g',[v]));

  //w := GetRobotW(0);
  SetRCValue(6, 2, format('%3.1g',[w]));

  if RCButtonPressed(4, 8) or UpdateMap then begin
    MapFromFile('map.txt');
    UpdateMap := False;
  end;

  if RCButtonPressed(5, 8) then begin
    ClearObstacles();
  end;
end;


procedure Initialize;
begin
  ClearButtons();


  t := 0;
  TrajControlMode := cmManual;

  v := 0;
  w:= 0;

  state := 0;

  side := 0.25;
  MapFromFile('map.txt');
  UpdateMap := False;

  SetRobotPos(0, 0, 0, 0.15, rad(90));
end;
