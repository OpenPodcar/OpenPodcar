using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Xml;
using System.Text;

namespace Pololu.Usc.Sequencer
{
    [Serializable]  // So that it can be put in the clipboard
    public class Frame
    {
        public string name;
        private ushort[] privateTargets;
    	public ushort length_ms;

        /// <summary>
        /// Gets the target of the given channel.
        /// </summary>
        /// <remarks>
        /// By retreiving targets this way, we protect the application against
        /// any kind of case where the Frame object might have fewer targets
        /// than expected.
        /// </remarks>
        [System.Runtime.CompilerServices.IndexerName("target")]
        public ushort this[int channel]
        {
            get
            {
                if (privateTargets == null || channel >= privateTargets.Length)
                {
                    return 0;
                }

                return privateTargets[channel];
            }
        }

        public ushort[] targets
        {
            set
            {
                privateTargets = value;
            }
        }

        /// <summary>
        /// Returns a string with all the servo positions, separated by spaces,
        /// e.g. "0 0 4000 0 1000 0 0".
        /// </summary>
        /// <returns></returns>
        public string getTargetsString()
        {
            string targetsString = "";
            for (int i = 0; i < privateTargets.Length; i++)
            {
                if (i != 0)
                {
                    targetsString += " ";
                }

                targetsString += privateTargets[i].ToString();
            }
            return targetsString;
        }

        /// <summary>
        /// Returns a string the name, duration, and all servo positions, separated by tabs.
        /// e.g. "Frame 1   500 0   0   0   4000    8000"
        /// </summary>
        /// <returns></returns>
        private string getTabSeparatedString()
        {
            string tabString = name + "\t" + length_ms;
            foreach (ushort target in privateTargets)
            {
                tabString += "\t" + target;
            }
            return tabString;
        }

        /// <summary>
        /// Take a (potentially malformed) string with target numbers separated by spaces
        /// and use it to set the targets.
        /// </summary>
        /// <param name="targetsString"></param>
        /// <param name="servoCount"></param>
        public void setTargetsFromString(string targetsString, byte servoCount)
        {
            ushort[] tmpTargets = new ushort[servoCount];

            string[] targetStrings = targetsString.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

            for (int i = 0; i < targetStrings.Length && i < servoCount; i++)
            {
                try
                {
                    tmpTargets[i] = ushort.Parse(targetStrings[i]);
                }
                catch { }
            }
            this.targets = tmpTargets;
        }

        public void writeXml(XmlWriter writer)
        {
            writer.WriteStartElement("Frame");
            writer.WriteAttributeString("name", name);
            writer.WriteAttributeString("duration", length_ms.ToString());
            writer.WriteString(getTargetsString());
            writer.WriteEndElement();
        }

        public static void copyToClipboard(List<Frame> frames)
        {
            if (frames.Count == 0)
            {
                return;
            }

            DataObject data = new DataObject();
            data.SetData(frames.ToArray());

            if (frames.Count == 1)
            {
                // Not necessary, but Microsoft recommends that we should
                // place data on the clipboard in as many formats possible.
                data.SetData(frames[0]);
            }

            StringBuilder sb = new StringBuilder();
            foreach (Frame frame in frames)
            {
                sb.AppendLine(frame.getTabSeparatedString());
            }
            data.SetText(sb.ToString());

            Clipboard.SetDataObject(data, true);
        }

        public static List<Frame> getFromClipboard()
        {
            Frame[] frameArray = Clipboard.GetData("Pololu.Usc.Sequencer.Frame[]") as Frame[];
            if (frameArray != null)
            {
                return new List<Frame>(frameArray);
            }

            Frame frameDirect = Clipboard.GetData("Pololu.Usc.Sequencer.Frame") as Frame;
            if (frameDirect != null)
            {
                return new List<Frame> { frameDirect };
            }

            if (Clipboard.ContainsText())
            {
                // Tab-separated-values for interop with spreadsheet programs.
                List<Frame> frames = new List<Frame>();
                String[] rows = Clipboard.GetText().Split('\n');
                foreach (String row in rows)
                {
                    String[] parts = row.Split('\t');
                    if (parts.Length < 3)
                    {
                        // There are not enough tab-separated parts available
                        // for name, duration, and one target so this line has
                        // no chance of being a frame.  Go to the next line.
                        continue;
                    }

                    Frame frame = new Frame();

                    frame.name = parts[0];
                    
                    // Prevent importing ridiculously long names.
                    if (frame.name.Length > 80)
                    {
                        frame.name = frame.name.Substring(0, 80);                        
                    }

                    try
                    {
                        frame.length_ms = ushort.Parse(parts[1]);
                    }
                    catch
                    {
                        frame.length_ms = 500;
                    }

                    List<ushort> targets = new List<ushort>();
                    for (int i = 2; i < parts.Length; i++)
                    {
                        try
                        {
                            targets.Add(ushort.Parse(parts[i]));
                        }
                        catch
                        {
                            targets.Add(0);
                        }
                    }

                    frame.targets = targets.ToArray();
                    frames.Add(frame);
                }
                return frames;
            }

            return null;
        }
    }
}