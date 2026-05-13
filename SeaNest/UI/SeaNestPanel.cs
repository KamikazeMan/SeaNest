using Eto.Drawing;
using Eto.Forms;
using Rhino;
using Rhino.UI;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Security.Cryptography;

namespace SeaNest
{
    [Guid("2cfcd53f-6e70-4333-8cd5-ffad5187fdb0")]
    public class SeaNestPanel : Panel, IPanel
    {
        public static Guid PanelId => typeof(SeaNestPanel).GUID;
        private WebView _webView;

        public SeaNestPanel()
        {
            _webView = new WebView();
            _webView.DocumentLoading += OnDocumentLoading;

            var html = BuildHtml(GetCurrentUnitSystem());
            _webView.LoadHtml(html);

            Content = _webView;

            RhinoDoc.DocumentPropertiesChanged += OnDocPropertiesChanged;
        }

        private void OnDocPropertiesChanged(object sender, DocumentEventArgs e)
        {
            try
            {
                var units = GetCurrentUnitSystem().ToUpper();
                _webView.ExecuteScript("if(document.getElementById('unit-display')) document.getElementById('unit-display').textContent='" + units + "';");
            }
            catch { }
        }

        /// <summary>
        /// WebView navigation interceptor. For seanest:// links, we cancel navigation,
        /// extract the command name, and DEFER its execution onto Eto's UI message pump.
        ///
        /// Why defer: invoking RhinoApp.RunScript synchronously here means a Rhino command
        /// (which may open another Eto Form like our ProgressDialog) starts running while
        /// the WebView is still mid-navigation, holding internal locks. The two Eto contexts
        /// can deadlock or crash. AsyncInvoke posts the work to the UI message queue, letting
        /// the WebView event finish cleanly first.
        /// </summary>
        private void OnDocumentLoading(object sender, WebViewLoadingEventArgs e)
        {
            var url = e.Uri?.ToString() ?? "";
            if (!url.StartsWith("seanest://")) return;

            e.Cancel = true;
            var command = url.Replace("seanest://", "").TrimEnd('/').Split('?')[0];

            Application.Instance.AsyncInvoke(() => HandleCommand(command));
        }

        private void HandleCommand(string command)
        {
            switch (command)
            {
                case "thicken": RhinoApp.RunScript("SeaNestThicken", false); break;
                case "slots": RhinoApp.RunScript("SeaNestSlots", false); break;
                case "ratholes": RhinoApp.RunScript("SeaNestRatHoles", false); break;
                case "trim": RhinoApp.RunScript("SeaNestTrim", false); break;
                case "renest": RhinoApp.RunScript("SeaNestReNest", false); break;
                case "export": RhinoApp.RunScript("SeaNestExport", false); break;
                case "plate": RhinoApp.RunScript("SeaNestPlate", false); break;
                case "nest": RhinoApp.RunScript("SeaNestNest", false); break;
                case "settings": RhinoApp.RunScript("SeaNestSettings", false); break;
                case "close": Panels.ClosePanel(PanelId); break;
                default: RhinoApp.WriteLine("SeaNest: Unknown command — " + command); break;
            }
        }

        private string GetCurrentUnitSystem()
        {
            var settings = SeaNestPlugin.Instance?.UserSettings;
            if (settings != null && settings.UnitSystem != "Auto")
                return settings.UnitSystem;

            var doc = RhinoDoc.ActiveDoc;
            if (doc == null) return "Inches";

            switch (doc.ModelUnitSystem)
            {
                case Rhino.UnitSystem.Inches: return "Inches";
                case Rhino.UnitSystem.Millimeters: return "Millimeters";
                case Rhino.UnitSystem.Centimeters: return "Centimeters";
                case Rhino.UnitSystem.Feet: return "Feet";
                case Rhino.UnitSystem.Meters: return "Meters";
                default: return doc.ModelUnitSystem.ToString();
            }
        }

        public void PanelShown(uint documentSerialNumber, ShowPanelReason reason)
        {
            RhinoApp.WriteLine("SeaNest panel opened.");
        }

        public void PanelHidden(uint documentSerialNumber, ShowPanelReason reason)
        {
            RhinoApp.WriteLine("SeaNest panel closed.");
        }

        public void PanelClosing(uint documentSerialNumber, bool onCloseDocument)
        {
        }

        // =============================================
        // THE FULL HTML UI
        // =============================================
        private string BuildHtml(string unitSystem)
        {
            return @"<!DOCTYPE html>
<html lang=""en"">
<head>
<meta charset=""UTF-8"">
<meta name=""viewport"" content=""width=device-width, initial-scale=1.0"">
<style>
  :root {
    --bg-deep: #0a0e14;
    --bg-panel: #10151d;
    --bg-card: #161c27;
    --bg-card-hover: #1c2535;
    --bg-card-active: #1a2233;
    --border-subtle: rgba(0,206,201,0.08);
    --border-mid: rgba(0,206,201,0.15);
    --border-bright: rgba(0,206,201,0.35);
    --cyan: #00cec9;
    --cyan-bright: #00fff7;
    --red-primary: #d63031;
    --red-bright: #ff4444;
    --text-primary: #e2e8f0;
    --text-secondary: #7b8a9e;
    --text-dim: #4a5568;
    --green-accent: #2ecc71;
    --orange-accent: #f39c12;
    --blue-accent: #4a9eff;
    --purple-accent: #a29bfe;
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family: 'Segoe UI', Tahoma, sans-serif;
    background: var(--bg-deep);
    color: var(--text-primary);
    padding: 0;
    overflow-x: hidden;
    user-select: none;
  }

  .panel {
    padding: 0;
    background: var(--bg-panel);
    min-height: 100vh;
    border: 1px solid var(--border-mid);
    position: relative;
  }

  /* === HEADER === */
  .header {
    padding: 20px 16px 14px;
    position: relative;
    overflow: hidden;
    border-bottom: 1px solid rgba(0,206,201,0.08);
  }
  .header-bg {
    position: absolute; inset:0; opacity:0.02;
    background-image: linear-gradient(45deg,#fff 1px,transparent 1px), linear-gradient(-45deg,#fff 1px,transparent 1px);
    background-size: 10px 10px;
    pointer-events: none;
  }
  .header-inner { display:flex; align-items:center; gap:14px; position:relative; z-index:1; }

  .badges { position:absolute; top:8px; right:12px; display:flex; gap:5px; z-index:2; }
  .badge { font-size:9px; color:var(--text-dim); background:rgba(0,206,201,0.05); border:1px solid var(--border-subtle); padding:2px 8px; border-radius:10px; font-weight:600; }
  .badge-pro { color:var(--cyan); border-color:rgba(0,206,201,0.2); background:rgba(0,206,201,0.08); }

  /* Radar */
  .radar-wrap { width:52px; height:52px; border-radius:50%; overflow:hidden; position:relative; flex-shrink:0;
    background: radial-gradient(circle, rgba(0,206,201,0.06) 0%, rgba(0,0,0,0.3) 100%);
    border: 1.5px solid rgba(0,206,201,0.2);
    box-shadow: 0 0 10px rgba(0,206,201,0.08), inset 0 0 8px rgba(0,0,0,0.4); }
  .radar-ring { position:absolute; border-radius:50%; border:1px solid rgba(0,206,201,0.1); top:50%; left:50%; transform:translate(-50%,-50%); }
  .rr1 { width:75%; height:75%; } .rr2 { width:50%; height:50%; } .rr3 { width:25%; height:25%; }
  .radar-cross-h,.radar-cross-v { position:absolute; background:rgba(0,206,201,0.07); }
  .radar-cross-h { width:100%; height:1px; top:50%; } .radar-cross-v { width:1px; height:100%; left:50%; }
  .radar-sweep { position:absolute; inset:0; animation: radarSweep 3s linear infinite; }
  .radar-sweep::before { content:''; position:absolute; top:50%; left:50%; width:50%; height:50%; transform-origin:0 0; transform:rotate(-90deg);
    background: conic-gradient(from 0deg, rgba(0,206,201,0.35) 0deg, rgba(0,206,201,0.08) 30deg, transparent 70deg); border-radius:0 100% 0 0; }
  @keyframes radarSweep { from{transform:rotate(0deg)} to{transform:rotate(360deg)} }
  .radar-blip { position:absolute; width:4px; height:4px; background:var(--cyan-bright); border-radius:50%;
    box-shadow: 0 0 5px var(--cyan), 0 0 10px rgba(0,206,201,0.3); animation: blipFade 3s linear infinite; }
  .rb1 { top:30%; left:62%; } .rb2 { top:58%; left:35%; animation-delay:-1.2s; opacity:0.7; } .rb3 { top:40%; left:40%; animation-delay:-2.1s; width:3px; height:3px; opacity:0.5; }
  @keyframes blipFade { 0%,10%{opacity:1;filter:brightness(1.5)} 50%{opacity:0.15} 90%,100%{opacity:0.05} }
  .radar-center { position:absolute; width:3px; height:3px; background:var(--cyan); border-radius:50%; top:50%; left:50%; transform:translate(-50%,-50%); box-shadow:0 0 4px var(--cyan); z-index:2; }

  .brand-name { font-weight:800; font-size:24px; line-height:1; letter-spacing:1px; }
  .brand-name .sea { color:var(--text-primary); }
  .brand-name .nest { color:var(--cyan); text-shadow:0 0 12px rgba(0,206,201,0.2); }
  .brand-tag { font-size:9.5px; font-weight:600; color:var(--text-secondary); letter-spacing:2.5px; text-transform:uppercase; margin-top:4px; }

  /* === STATUS BAR === */
  .status-bar { background:rgba(0,206,201,0.03); padding:8px 16px; display:flex; align-items:center; justify-content:space-between; font-size:11px; color:var(--text-secondary);
    border-bottom:1px solid var(--border-subtle); position:relative; }
  .status-bar::before { content:''; position:absolute; inset:0; background:repeating-linear-gradient(0deg,transparent,transparent 2px,rgba(0,206,201,0.006) 2px,rgba(0,206,201,0.006) 4px); pointer-events:none; }
  .status-left { display:flex; align-items:center; gap:6px; }
  .status-dot { width:6px; height:6px; background:var(--green-accent); border-radius:50%; box-shadow:0 0 4px rgba(46,204,113,0.5); animation:dotPulse 2.5s ease-in-out infinite; }
  @keyframes dotPulse { 0%,100%{box-shadow:0 0 4px rgba(46,204,113,0.5)} 50%{box-shadow:0 0 8px rgba(46,204,113,0.7)} }
  .status-unit { font-weight:700; color:var(--cyan); font-size:10px; letter-spacing:1.5px; }

  /* === SECTION LABELS === */
  .section-label { font-size:9px; font-weight:700; color:var(--text-dim); letter-spacing:2.5px; text-transform:uppercase; padding:10px 16px 6px; }

  /* === TOOL BUTTONS === */
  .tool-grid { display:grid; grid-template-columns:1fr 1fr; gap:6px; padding:0 12px 8px; }

  .tool-btn {
    background: var(--bg-card);
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    padding: 12px 10px;
    cursor: pointer;
    transition: all 0.2s ease;
    display: flex;
    align-items: center;
    gap: 9px;
    position: relative;
    overflow: hidden;
  }
  .tool-btn::before { content:''; position:absolute; left:0; top:0; bottom:0; width:3px; background:var(--accent); opacity:0; transition:opacity 0.2s; }
  .tool-btn::after { content:''; position:absolute; inset:0; background:linear-gradient(90deg,transparent,rgba(0,206,201,0.03),transparent); left:-100%; transition:left 0.5s; pointer-events:none; }
  .tool-btn:hover { background:var(--bg-card-hover); border-color:var(--border-bright);
    box-shadow: 0 0 6px rgba(0,206,201,0.1), 0 0 16px rgba(0,206,201,0.05); transform:translateY(-1px); }
  .tool-btn:hover::before { opacity:1; }
  .tool-btn:hover::after { left:100%; }
  .tool-btn:active { transform:translateY(1px) scale(0.98); background:var(--bg-card-active); box-shadow:inset 0 2px 6px rgba(0,0,0,0.3); transition-duration:0.05s; }
  .tool-btn.active { border-color:var(--cyan); background:rgba(0,206,201,0.06); box-shadow:0 0 8px rgba(0,206,201,0.1); }
  .tool-btn.active::before { opacity:1; }

  .tool-icon { width:32px; height:32px; border-radius:7px; display:flex; align-items:center; justify-content:center; flex-shrink:0;
    background: var(--icon-bg); color: var(--accent); transition:all 0.2s; }
  .tool-btn:hover .tool-icon { box-shadow:0 0 8px var(--icon-glow); }
  .tool-icon svg { width:16px; height:16px; }
  .tool-label { font-weight:600; font-size:11.5px; color:var(--text-primary); line-height:1.25; }
  .tool-shortcut { font-size:9px; color:var(--text-dim); font-weight:500; margin-top:1px; }

  /* === HERO BUTTONS (Thicken + Nest) === */
  .hero-btn {
    margin: 0 12px 6px;
    padding: 13px 14px;
    border-radius: 7px;
    cursor: pointer;
    display: flex;
    align-items: center;
    gap: 12px;
    position: relative;
    overflow: hidden;
    transition: all 0.25s;
    border: 1px solid rgba(0,206,201,0.18);
    background: linear-gradient(135deg, rgba(0,206,201,0.06), rgba(0,206,201,0.02));
  }
  .hero-btn .shimmer { position:absolute; inset:0; background:linear-gradient(105deg,transparent 40%,rgba(0,206,201,0.04) 50%,transparent 60%); background-size:200% 100%; animation:shimmer 4s ease-in-out infinite; pointer-events:none; }
  @keyframes shimmer { 0%,100%{background-position:200% 0} 50%{background-position:-200% 0} }
  .hero-btn:hover { border-color:rgba(0,206,201,0.4); box-shadow:0 0 8px rgba(0,206,201,0.12), 0 0 20px rgba(0,206,201,0.06); transform:translateY(-1px); }
  .hero-btn:active { transform:translateY(1px) scale(0.99); box-shadow:inset 0 2px 6px rgba(0,0,0,0.3); transition-duration:0.05s; }
  .hero-icon { width:36px; height:36px; border-radius:8px; background:rgba(0,206,201,0.1); display:flex; align-items:center; justify-content:center; color:var(--cyan); z-index:1; }
  .hero-icon svg { width:18px; height:18px; }
  .hero-text { z-index:1; }
  .hero-label { font-weight:700; font-size:13px; color:var(--text-primary); letter-spacing:0.3px; }
  .hero-desc { font-size:10px; color:var(--text-secondary); margin-top:1px; }
  .hero-arrow { margin-left:auto; color:var(--text-dim); z-index:1; transition:all 0.2s; }
  .hero-btn:hover .hero-arrow { color:var(--cyan); transform:translateX(3px); }

  /* === ACTIVITY LOG === */
  .log-wrap { margin:0 12px 8px; background:rgba(0,0,0,0.2); border:1px solid var(--border-subtle); border-radius:5px; padding:6px 10px; max-height:64px; overflow:hidden; }
  .log-entry { font-size:10px; color:var(--text-dim); line-height:1.8; display:flex; gap:6px; }
  .log-time { color:var(--text-dim); font-weight:500; min-width:36px; }
  .log-action { color:var(--cyan); font-weight:600; }
  .log-detail { color:var(--text-secondary); }

  /* === FOOTER === */
  .footer { padding:8px 12px 12px; display:flex; gap:6px; }
  .footer-btn { flex:1; padding:9px; border-radius:6px; cursor:pointer; font-weight:600; font-size:11.5px; display:flex; align-items:center; justify-content:center; gap:6px; transition:all 0.2s; border:1px solid var(--border-subtle); }
  .footer-btn svg { width:13px; height:13px; }
  .btn-settings { background:var(--bg-card); color:var(--text-secondary); }
  .btn-settings:hover { background:var(--bg-card-hover); color:var(--text-primary); border-color:var(--border-bright); }
  .btn-close { background:rgba(214,48,49,0.06); color:var(--red-primary); border-color:rgba(214,48,49,0.12); }
  .btn-close:hover { background:rgba(214,48,49,0.12); color:var(--red-bright); border-color:rgba(214,48,49,0.3); }
  .footer-btn:active { transform:scale(0.97); }

  /* === RIPPLE === */
  .ripple { position:absolute; border-radius:50%; background:rgba(0,206,201,0.12); transform:scale(0); animation:rip 0.5s ease-out; pointer-events:none; }
  @keyframes rip { to{transform:scale(3);opacity:0} }

  /* === ENTRANCE === */
  @keyframes fadeUp { from{opacity:0;transform:translateY(12px)} to{opacity:1;transform:translateY(0)} }
  .anim1 { animation:fadeUp 0.4s ease backwards; }
  .anim2 { animation:fadeUp 0.4s ease 0.05s backwards; }
  .anim3 { animation:fadeUp 0.4s ease 0.1s backwards; }
  .anim4 { animation:fadeUp 0.4s ease 0.15s backwards; }
  .anim5 { animation:fadeUp 0.4s ease 0.2s backwards; }
  .anim6 { animation:fadeUp 0.4s ease 0.25s backwards; }
  .anim7 { animation:fadeUp 0.4s ease 0.3s backwards; }
  .anim8 { animation:fadeUp 0.4s ease 0.35s backwards; }
</style>
</head>
<body>
<div class=""panel"">

  <!-- HEADER -->
  <div class=""header anim1"">
    <div class=""header-bg""></div>
    <div class=""badges""><span class=""badge badge-pro"">PRO</span><span class=""badge"">v1.0</span></div>
    <div class=""header-inner"">
      <div class=""radar-wrap"">
        <div class=""radar-ring rr1""></div><div class=""radar-ring rr2""></div><div class=""radar-ring rr3""></div>
        <div class=""radar-cross-h""></div><div class=""radar-cross-v""></div>
        <div class=""radar-sweep""></div>
        <div class=""radar-blip rb1""></div><div class=""radar-blip rb2""></div><div class=""radar-blip rb3""></div>
        <div class=""radar-center""></div>
      </div>
      <div>
        <div class=""brand-name""><span class=""sea"">Sea</span><span class=""nest"">Nest</span></div>
        <div class=""brand-tag"">Precision Automation</div>
      </div>
    </div>
  </div>

  <!-- STATUS -->
  <div class=""status-bar anim2"">
    <div class=""status-left""><div class=""status-dot""></div><span>Rhino 8 Connected</span></div>
    <span class=""status-unit"" id=""unit-display"">" + unitSystem.ToUpper() + @"</span>
  </div>

  <!-- FABRICATION -->
  <div class=""section-label anim3"">Fabrication</div>
  <div class=""hero-btn anim3"" onclick=""nav('thicken')"">
    <div class=""shimmer""></div>
    <div class=""hero-icon""><svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2"" stroke-linecap=""round""><path d=""M12 2v4M12 18v4M4.93 4.93l2.83 2.83M16.24 16.24l2.83 2.83M2 12h4M18 12h4M4.93 19.07l2.83-2.83M16.24 7.76l2.83-2.83""/></svg></div>
    <div class=""hero-text""><div class=""hero-label"">Smart Thickening</div><div class=""hero-desc"">Add material thickness to surfaces</div></div>
    <div class=""hero-arrow""><svg width=""14"" height=""14"" viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><path d=""M9 18l6-6-6-6""/></svg></div>
  </div>

  <!-- TOOLS -->
  <div class=""section-label anim4"">Tools</div>
  <div class=""tool-grid"">
    <div class=""tool-btn anim4"" style=""--accent:#d63031;--icon-bg:rgba(214,48,49,0.08);--icon-glow:rgba(214,48,49,0.12)"" onclick=""nav('slots')"">
      <div class=""tool-icon""><svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><path d=""M10 3H6a2 2 0 00-2 2v4""/><path d=""M14 3h4a2 2 0 012 2v4""/><path d=""M10 21H6a2 2 0 01-2-2v-4""/><path d=""M14 21h4a2 2 0 002-2v-4""/><line x1=""9"" y1=""12"" x2=""15"" y2=""12""/></svg></div>
      <div><div class=""tool-label"">Slots</div><div class=""tool-shortcut"">Ctrl+I</div></div>
    </div>
    <div class=""tool-btn anim4"" style=""--accent:#f39c12;--icon-bg:rgba(243,156,18,0.08);--icon-glow:rgba(243,156,18,0.12)"" onclick=""nav('ratholes')"">
      <div class=""tool-icon""><svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><circle cx=""12"" cy=""12"" r=""3""/><circle cx=""12"" cy=""12"" r=""8"" stroke-dasharray=""4 3""/></svg></div>
      <div><div class=""tool-label"">Cut Rat Holes</div><div class=""tool-shortcut"">Ctrl+R</div></div>
    </div>
    <div class=""tool-btn anim5"" style=""--accent:#4a9eff;--icon-bg:rgba(74,158,255,0.08);--icon-glow:rgba(74,158,255,0.12)"" onclick=""nav('trim')"">
      <div class=""tool-icon""><svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><circle cx=""6"" cy=""6"" r=""3""/><path d=""M8.12 8.12L12 12""/><path d=""M20 4L8.12 15.88""/><circle cx=""6"" cy=""18"" r=""3""/><path d=""M14.8 14.8L20 20""/></svg></div>
      <div><div class=""tool-label"">Auto Trimming</div><div class=""tool-shortcut"">Ctrl+T</div></div>
    </div>
    <div class=""tool-btn anim5"" style=""--accent:#a29bfe;--icon-bg:rgba(162,155,254,0.08);--icon-glow:rgba(162,155,254,0.12)"" onclick=""nav('plate')"">
      <div class=""tool-icon""><svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><rect x=""2"" y=""6"" width=""20"" height=""12"" rx=""2""/><line x1=""12"" y1=""6"" x2=""12"" y2=""18"" stroke-dasharray=""2 2""/><line x1=""7"" y1=""6"" x2=""7"" y2=""18"" opacity=""0.4"" stroke-dasharray=""2 2""/></svg></div>
      <div><div class=""tool-label"">Custom Plate</div><div class=""tool-shortcut"">Ctrl+P</div></div>
    </div>
  </div>

  <!-- NESTING -->
  <div class=""section-label anim7"">Nesting & Export</div>
  <div class=""hero-btn anim7"" onclick=""nav('nest')"">
    <div class=""shimmer""></div>
    <div class=""hero-icon""><svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><rect x=""2"" y=""2"" width=""8"" height=""8"" rx=""1""/><rect x=""14"" y=""2"" width=""8"" height=""6"" rx=""1""/><rect x=""2"" y=""14"" width=""6"" height=""8"" rx=""1""/><rect x=""12"" y=""12"" width=""10"" height=""10"" rx=""1""/></svg></div>
    <div class=""hero-text""><div class=""hero-label"">Nest</div><div class=""hero-desc"">Flatten plates and nest onto sheets</div></div>
    <div class=""hero-arrow""><svg width=""14"" height=""14"" viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><path d=""M9 18l6-6-6-6""/></svg></div>
  </div>
  <div class=""hero-btn anim7"" onclick=""nav('renest')"">
    <div class=""shimmer""></div>
    <div class=""hero-icon""><svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2"" stroke-linecap=""round"" stroke-linejoin=""round""><path d=""M21 12a9 9 0 11-3-6.7""/><polyline points=""21 3 21 9 15 9""/></svg></div>
    <div class=""hero-text""><div class=""hero-label"">Re-Nest</div><div class=""hero-desc"">Re-nest existing curves with new parameters</div></div>
    <div class=""hero-arrow""><svg width=""14"" height=""14"" viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><path d=""M9 18l6-6-6-6""/></svg></div>
  </div>
  <div class=""hero-btn anim7"" onclick=""nav('export')"">
    <div class=""shimmer""></div>
    <div class=""hero-icon""><svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2"" stroke-linecap=""round"" stroke-linejoin=""round""><path d=""M21 15v4a2 2 0 01-2 2H5a2 2 0 01-2-2v-4""/><polyline points=""7 10 12 15 17 10""/><line x1=""12"" y1=""15"" x2=""12"" y2=""3""/></svg></div>
    <div class=""hero-text""><div class=""hero-label"">Export</div><div class=""hero-desc"">Save nested output as DXF or DWG</div></div>
    <div class=""hero-arrow""><svg width=""14"" height=""14"" viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><path d=""M9 18l6-6-6-6""/></svg></div>
  </div>

  <!-- LOG -->
  <div class=""section-label anim8"">Recent</div>
  <div class=""log-wrap anim8"" id=""logWrap"">
    <div class=""log-entry""><span class=""log-time"">--:--</span><span class=""log-action"">Ready</span><span class=""log-detail"">— select a tool to begin</span></div>
  </div>

  <!-- FOOTER -->
  <div class=""footer anim8"">
    <div class=""footer-btn btn-settings"" onclick=""nav('settings')"">
      <svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2""><circle cx=""12"" cy=""12"" r=""3""/><path d=""M19.4 15a1.65 1.65 0 00.33 1.82l.06.06a2 2 0 010 2.83 2 2 0 01-2.83 0l-.06-.06a1.65 1.65 0 00-1.82-.33 1.65 1.65 0 00-1 1.51V21a2 2 0 01-4 0v-.09A1.65 1.65 0 009 19.4a1.65 1.65 0 00-1.82.33l-.06.06a2 2 0 01-2.83-2.83l.06-.06A1.65 1.65 0 004.68 15a1.65 1.65 0 00-1.51-1H3a2 2 0 010-4h.09A1.65 1.65 0 004.6 9a1.65 1.65 0 00-.33-1.82l-.06-.06a2 2 0 112.83-2.83l.06.06A1.65 1.65 0 009 4.68a1.65 1.65 0 001-1.51V3a2 2 0 014 0v.09a1.65 1.65 0 001 1.51 1.65 1.65 0 001.82-.33l.06-.06a2 2 0 012.83 2.83l-.06.06A1.65 1.65 0 0019.4 9a1.65 1.65 0 001.51 1H21a2 2 0 010 4h-.09a1.65 1.65 0 00-1.51 1z""/></svg>
      Settings
    </div>
    <div class=""footer-btn btn-close"" onclick=""nav('close')"">
      <svg viewBox=""0 0 24 24"" fill=""none"" stroke=""currentColor"" stroke-width=""2.5""><line x1=""18"" y1=""6"" x2=""6"" y2=""18""/><line x1=""6"" y1=""6"" x2=""18"" y2=""18""/></svg>
      Close
    </div>
  </div>

</div>

<script>
function nav(cmd) {
  // Add log entry
  var log = document.getElementById('logWrap');
  var now = new Date();
  var t = now.getHours().toString().padStart(2,'0') + ':' + now.getMinutes().toString().padStart(2,'0');
  var names = {thicken:'Thicken',slots:'Slots',ratholes:'Rat Holes',trim:'Trim',plate:'Plate',nest:'Nest',renest:'Re-Nest',export:'Export',settings:'Settings',close:'Close'};
  var entry = document.createElement('div');
  entry.className = 'log-entry';
  entry.innerHTML = '<span class=""log-time"">' + t + '</span><span class=""log-action"">' + (names[cmd]||cmd) + '</span><span class=""log-detail"">— activated</span>';
  log.insertBefore(entry, log.firstChild);

  // Navigate to trigger C# handler
  window.location.href = 'seanest://' + cmd;
}

// Click sound
var AudioCtx = window.AudioContext || window.webkitAudioContext;
var actx;
function clickSound() {
  try {
    if(!actx) actx = new AudioCtx();
    var o = actx.createOscillator(), g = actx.createGain();
    o.connect(g); g.connect(actx.destination);
    o.frequency.setValueAtTime(1200, actx.currentTime);
    o.frequency.exponentialRampToValueAtTime(700, actx.currentTime+0.06);
    g.gain.setValueAtTime(0.06, actx.currentTime);
    g.gain.exponentialRampToValueAtTime(0.001, actx.currentTime+0.06);
    o.start(); o.stop(actx.currentTime+0.06);
  } catch(e){}
}

// Add click sound + ripple to all buttons
document.querySelectorAll('.tool-btn,.hero-btn,.footer-btn').forEach(function(el) {
  el.addEventListener('mousedown', function(e) {
    clickSound();
    var r = document.createElement('span');
    r.className = 'ripple';
    var rect = el.getBoundingClientRect();
    var sz = Math.max(rect.width, rect.height);
    r.style.width = r.style.height = sz + 'px';
    r.style.left = (e.clientX - rect.left - sz/2) + 'px';
    r.style.top = (e.clientY - rect.top - sz/2) + 'px';
    el.appendChild(r);
    r.addEventListener('animationend', function(){ r.remove(); });
  });
});
</script>
</body>
</html>";
        }
    }
}
