
-- Pull in the wezterm API
local wezterm = require 'wezterm'

local act = wezterm.action
-- This will hold the configuration.
local config = wezterm.config_builder()
config.default_prog = { 'powershell.exe', '-NoLogo' }
-- This is where you actually apply your config choices.
config.font =  wezterm.font('Berkeley Mono')
-- config.font =  wezterm.font('Berkeley Mono', { weight = 'Regular' })
-- config.font =  wezterm.font('Departure Mono')
-- config.font = wezterm.font("Ioskeley Mono", {weight = "Regular"})
config.font_size = 12.5
-- config.color_scheme = 'Apprentice (Gogh)'
-- config.color_scheme = 'Atelier Estuary Light (base16)'
-- config.color_scheme = 'Atlas (base16)'
-- config.color_scheme = 'Aura (Gogh)'
config.color_scheme = 'Bamboo'
-- config.color_scheme = 'Bim (Gogh)'
-- config.color_scheme = 'Banana Blueberry'
-- config.color_scheme = 'darkmoss (base16)'
-- config.color_scheme = 'dawnfox'
-- config.color_scheme = 'Ef-Arbutus'
-- config.color_scheme = 'Vaughn'
-- config.color_scheme = 'Jellybeans'
 -- config.color_scheme = 'lovelace'
 config.color_scheme = 'duskfox'
--config.font_size = 11.5
-- config.show_tabs_in_tab_bar = false
config.enable_tab_bar = false
-- config.hide_tab_bar_if_only_one_tab = true
config.window_close_confirmation = 'NeverPrompt'
-- For example, changing the initial geometry for new windows:
config.initial_cols = 100
config.initial_rows = 50

 config.font_rules = {
  {
    intensity = "Bold",
    font = wezterm.font("Berkeley Mono", { weight = "Regular" }),
  },
  {
    italic = true,
    font = wezterm.font("Berkeley Mono", { italic = false }),
  },
}

config.window_padding = {
  left = 2,
  right = 0,
  top = 2,
  bottom = 0,
}

wezterm.on('spawn-tab-same-workspace', function(window, pane)
  -- capturamos el workspace actual
  local workspace = window:active_workspace()
  local path = pane:get_current_working_dir()
  wezterm.log_info("current workspace:", path)

  -- forzamos el workspace antes de crear el tab
  window:perform_action(
    act.SwitchToWorkspace { name = workspace },
    pane
  )

  -- ahora sí: spawn tab
  window:perform_action(
    act.SpawnTab 'CurrentPaneDomain',
    pane
  )
end)
-- or, changing the font size and color scheme.

local act = wezterm.action
config.keys = {
  {
    key = 't',
    mods = 'CTRL',
    action = act.EmitEvent 'spawn-tab-same-workspace',
    -- action = act.SpawnTab 'CurrentPaneDomain'
    -- action = wezterm.action_callback(function(window, pane)

    --     local cwd_uri = pane:get_current_working_dir()
    --     local cwd = nil

    --     if cwd_uri then
    --       cwd = cwd_uri.file_path
    --     end

    --     wezterm.log_info("cwd path: ", cwd)
    --     window:perform_action(
    --       act.SpawnCommandInNewTab {
    --         cwd = cwd,
    --       },
    --       pane
    --     )
    --   end),
    },
  {
    key = '[',
    mods = 'CTRL',
    action = act.ActivateTabRelative(-1)
  },
  {
    key = ']',
    mods = 'CTRL',
    action = act.ActivateTabRelative(1)
  },
}

-- Finally, return the configuration to wezterm:
return config
