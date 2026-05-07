using System;
using Eto.Drawing;
using Eto.Forms;

namespace SeaNest.RhinoAdapters
{
    /// <summary>
    /// Borderless progress dialog shown during a nesting run.
    ///
    /// Implemented as a non-modal <see cref="Form"/> (not a <see cref="Dialog"/>) so the
    /// command thread can drive both the engine and the UI updates without deadlocking.
    /// Centered on the primary screen, Topmost so it floats above Rhino.
    /// </summary>
    public sealed class ProgressDialog : Form
    {
        private readonly Label _statusLabel;

        public ProgressDialog(string initialMessage = "Nesting... Please Wait")
        {
            WindowStyle = WindowStyle.None;
            ShowInTaskbar = false;
            Topmost = true;
            Resizable = false;
            Maximizable = false;
            Minimizable = false;
            ShowActivated = true;

            BackgroundColor = Colors.White;

            _statusLabel = new Label
            {
                Text = initialMessage,
                TextAlignment = TextAlignment.Center,
                VerticalAlignment = VerticalAlignment.Center,
                Font = Fonts.Sans(12f, FontStyle.Bold)
            };

            Content = new StackLayout
            {
                Orientation = Orientation.Vertical,
                HorizontalContentAlignment = HorizontalAlignment.Center,
                VerticalContentAlignment = VerticalAlignment.Center,
                Padding = new Padding(30, 24),
                Spacing = 6,
                Items = { _statusLabel }
            };

            ClientSize = new Size(360, 90);

            // Center on primary screen.
            var screenBounds = Screen.PrimaryScreen.Bounds;
            int x = (int)(screenBounds.X + (screenBounds.Width - ClientSize.Width) / 2);
            int y = (int)(screenBounds.Y + (screenBounds.Height - ClientSize.Height) / 2);
            Location = new Point(x, y);
        }

        /// <summary>
        /// Update the status text. Safe to call from the command thread; if a UI dispatch
        /// is needed on a particular platform, Eto handles it internally for simple property sets.
        /// </summary>
        public void UpdateStatus(string message)
        {
            if (_statusLabel == null) return;

            Application.Instance.Invoke(() =>
            {
                _statusLabel.Text = message ?? string.Empty;
            });
        }
    }
}