function cap = cargar_captura(archivo)
%CARGAR_CAPTURA Lee una captura de VibraScan (CSV + JSON) para analizarla en MATLAB.
%
%   cap = cargar_captura('ruta/captura_20261008_221540')
%   cap = cargar_captura('ruta/captura_20261008_221540.csv')   (o .json)
%
%   Las capturas las escriben pc/captura.py y pc/monitor_vivo.py (formato en
%   pc/vibrascan_captura.py). Funciona en MATLAB R2015b (sin jsondecode: lee
%   los campos del JSON con expresiones regulares); en versiones con
%   jsondecode lo usa.
%
%   Campos de cap:
%     t          tiempo de cada muestra en s (reloj del micro)
%     n          indice de la muestra que da el firmware
%     seg        segmento de cada muestra (0, 1, ...). Un segmento es un tramo
%                de muestras contiguas con ODR y rango fijos: una FFT no debe
%                cruzar de un segmento a otro (habria un hueco en el medio).
%     ax, ay, az aceleracion en g (vectores columna)
%     cuentas    [x y z] en cuentas crudas del sensor
%     fs         tasa de muestreo MEDIDA en Hz (no el ODR nominal: el reloj del
%                sensor se aparta hasta ~0,7 %; ver la etapa 5 del README)
%     escala     g por cuenta (0,0039)
%     odr, rango ODR nominal (Hz) y rango (g)
%     nota       texto libre de la captura
%     segmentos  struct array: seg, motivo, n0, t_s, odr, rango, muestras, periodo_us
%     filtro     si existe <base>_filtro.csv (salida del filtro adaptativo del
%                micro): struct con n, y (componente realzada) y e (residuo), en g.
%                Si no existe, vacio.
%     meta_txt   el JSON completo como texto
%
%   Ejemplo (FFT de un segmento): ver ejemplo_fft.m

    base = archivo;
    [p, nom, ext] = fileparts(archivo);
    if any(strcmpi(ext, {'.csv', '.json'}))
        base = fullfile(p, nom);
    end

    % ---- Datos: seg,n,t_s,x,y,z (una fila por muestra) --------------------
    d = csvread([base '.csv'], 1, 0);
    cap.seg = d(:, 1);
    cap.n = d(:, 2);
    cap.t = d(:, 3);
    cap.cuentas = d(:, 4:6);

    % ---- Metadatos -------------------------------------------------------
    txt = fileread([base '.json']);
    cap.meta_txt = txt;
    if exist('jsondecode') %#ok<EXIST> (R2016b o posterior)
        m = jsondecode(txt);
        cap.escala = valor_o(m, 'escala_g_por_cuenta', 0.0039);
        cap.odr = valor_o(m, 'odr_hz', NaN);
        cap.rango = valor_o(m, 'rango_g', NaN);
        cap.nota = valor_o(m, 'nota', '');
        periodo = valor_o(m, 'periodo_us', NaN);
        cap.segmentos = valor_o(m, 'segmentos', struct([]));
    else
        cap.escala = num_json(txt, 'escala_g_por_cuenta', 0.0039);
        cap.odr = num_json(txt, 'odr_hz', NaN);
        cap.rango = num_json(txt, 'rango_g', NaN);
        cap.nota = str_json(txt, 'nota');
        periodo = num_json(cabecera(txt), 'periodo_us', NaN);
        cap.segmentos = segmentos_json(txt);
    end
    if isempty(periodo) || isnan(periodo)
        % Sin periodo en el JSON (captura cortada): se estima con t
        periodo = median(diff(cap.t)) * 1e6;
    end
    cap.fs = 1e6 / periodo;

    cap.ax = cap.cuentas(:, 1) * cap.escala;
    cap.ay = cap.cuentas(:, 2) * cap.escala;
    cap.az = cap.cuentas(:, 3) * cap.escala;

    % ---- Salida del filtro adaptativo (opcional) ----------------------------
    cap.filtro = [];
    archivo_filtro = [base '_filtro.csv'];
    if exist(archivo_filtro, 'file')
        f = csvread(archivo_filtro, 1, 0);      % n,y,e
        cap.filtro.n = f(:, 1);
        cap.filtro.y = f(:, 2);
        cap.filtro.e = f(:, 3);
    end
end

% ---- Auxiliares (lectura del JSON sin jsondecode) ----------------------------

function v = valor_o(s, campo, defecto)
    if isfield(s, campo) && ~isempty(s.(campo))
        v = s.(campo);
    else
        v = defecto;
    end
end

function v = num_json(txt, campo, defecto)
    % Primer "campo": numero  (null o ausente -> defecto)
    tok = regexp(txt, ['"' campo '"\s*:\s*(-?[0-9.eE+]+)'], 'tokens', 'once');
    if isempty(tok)
        v = defecto;
    else
        v = str2double(tok{1});
    end
end

function v = str_json(txt, campo)
    tok = regexp(txt, ['"' campo '"\s*:\s*"((?:[^"\\]|\\.)*)"'], 'tokens', 'once');
    if isempty(tok)
        v = '';
    else
        v = tok{1};
    end
end

function c = cabecera(txt)
    % Texto del JSON antes de "segmentos": los campos de nivel superior.
    % (Los segmentos tambien tienen "periodo_us", y no deben confundirse.)
    k = strfind(txt, '"segmentos"');
    if isempty(k)
        c = txt;
    else
        c = txt(1:k(1) - 1);
    end
end

function segs = segmentos_json(txt)
    % Lista "segmentos": [ {...}, {...} ]: se toma el arreglo y cada objeto
    segs = struct('seg', {}, 'motivo', {}, 'n0', {}, 't_s', {}, 'odr', {}, ...
                  'rango', {}, 'muestras', {}, 'periodo_us', {});
    k = strfind(txt, '"segmentos"');
    if isempty(k)
        return;
    end
    i0 = k(1) + strfind(txt(k(1):end), '[');
    i0 = i0(1);
    nivel = 1;
    i = i0;
    while i <= numel(txt) && nivel > 0          % corchete que cierra el arreglo
        if txt(i) == '['
            nivel = nivel + 1;
        elseif txt(i) == ']'
            nivel = nivel - 1;
        end
        i = i + 1;
    end
    arreglo = txt(i0:i - 2);
    objetos = regexp(arreglo, '\{[^{}]*\}', 'match');
    for j = 1:numel(objetos)
        o = objetos{j};
        segs(j).seg = num_json(o, 'seg', NaN);
        segs(j).motivo = str_json(o, 'motivo');
        segs(j).n0 = num_json(o, 'n0', NaN);
        segs(j).t_s = num_json(o, 't_s', NaN);
        segs(j).odr = num_json(o, 'odr', NaN);
        segs(j).rango = num_json(o, 'rango', NaN);
        segs(j).muestras = num_json(o, 'muestras', NaN);
        segs(j).periodo_us = num_json(o, 'periodo_us', NaN);
    end
end
