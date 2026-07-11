using System.Diagnostics;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace HoloTile.Editor
{
    public static class OpenSimExportMenu
    {
        const string Python = @"E:\conda\envs\opensim_env\python.exe";
        const string Script = @"E:\OpenSim\scripts\export_walk_replay_for_unity.py";

        [MenuItem("HoloTile/Export OpenSim Walk Replay")]
        public static void ExportWalkReplay()
        {
            if (!File.Exists(Python))
            {
                EditorUtility.DisplayDialog("OpenSim export",
                    $"Python not found:\n{Python}\n\nUpdate path in OpenSimExportMenu.cs", "OK");
                return;
            }

            var psi = new ProcessStartInfo(Python, $"\"{Script}\"")
            {
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
            };
            using var proc = Process.Start(psi);
            string stdout = proc.StandardOutput.ReadToEnd();
            string stderr = proc.StandardError.ReadToEnd();
            proc.WaitForExit();

            AssetDatabase.Refresh();
            if (proc.ExitCode == 0)
                UnityEngine.Debug.Log($"[HoloTile] OpenSim export OK.\n{stdout}");
            else
                UnityEngine.Debug.LogError($"[HoloTile] OpenSim export failed.\n{stderr}\n{stdout}");
        }
    }
}
