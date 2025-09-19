import pygame
import sys
from config import *

class MainMenu:
    def __init__(self):
        """Initialize the main menu"""
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("2D Car Simulator - Main Menu")
        
        # Fonts
        self.title_font = pygame.font.Font(None, 64)
        self.menu_font = pygame.font.Font(None, 36)
        self.subtitle_font = pygame.font.Font(None, 24)
        
        # Menu options
        self.menu_options = [
            "Map 1: Default Layout (with parking spaces)",
            "Map 2: Empty Layout",
            "Map 3: Minimal Layout",
            "Quit"
        ]
        
        self.selected_option = 0
        self.clock = pygame.time.Clock()
        
    def draw(self):
        """Draw the menu"""
        self.screen.fill(WHITE)
        
        # Title
        title_text = self.title_font.render("2D Car Simulator", True, BLACK)
        title_rect = title_text.get_rect(center=(SCREEN_WIDTH // 2, 100))
        self.screen.blit(title_text, title_rect)
        
        # Subtitle
        subtitle_text = self.subtitle_font.render("Choose a map layout to start simulation", True, GREY)
        subtitle_rect = subtitle_text.get_rect(center=(SCREEN_WIDTH // 2, 150))
        self.screen.blit(subtitle_text, subtitle_rect)
        
        # Menu options
        start_y = 250
        for i, option in enumerate(self.menu_options):
            color = BLACK if i == self.selected_option else GREY
            
            # Highlight selected option
            if i == self.selected_option:
                highlight_rect = pygame.Rect(50, start_y + i * 60 - 10, SCREEN_WIDTH - 100, 50)
                pygame.draw.rect(self.screen, (240, 240, 240), highlight_rect)
                pygame.draw.rect(self.screen, BLACK, highlight_rect, 2)
            
            text = self.menu_font.render(option, True, color)
            text_rect = text.get_rect(center=(SCREEN_WIDTH // 2, start_y + i * 60 + 10))
            self.screen.blit(text, text_rect)
        
        # Controls info
        controls_y = start_y + len(self.menu_options) * 60 + 50
        controls_text = [
            "Simulation Controls:",
            "Q - Set start position",
            "A - Auto-navigate to parking (carrot method)",
            "S - Auto-navigate to parking (cross-track method)",
            "C - Clear path",
            "SPACE - Stop autopilot",
            "ESC - Return to menu"
        ]
        
        for i, text in enumerate(controls_text):
            color = BLACK if i == 0 else GREY
            font = self.subtitle_font if i == 0 else self.subtitle_font
            rendered_text = font.render(text, True, color)
            text_rect = rendered_text.get_rect(center=(SCREEN_WIDTH // 2, controls_y + i * 25))
            self.screen.blit(rendered_text, text_rect)
        
        pygame.display.flip()
    
    def handle_input(self, event):
        """Handle menu input"""
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                self.selected_option = (self.selected_option - 1) % len(self.menu_options)
            elif event.key == pygame.K_DOWN:
                self.selected_option = (self.selected_option + 1) % len(self.menu_options)
            elif event.key == pygame.K_RETURN:
                return self.selected_option
            elif event.key == pygame.K_ESCAPE:
                return 3  # Quit option
        return None
    
    def run(self):
        """Run the main menu loop"""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                
                selection = self.handle_input(event)
                if selection is not None:
                    if selection == 3:  # Quit
                        pygame.quit()
                        sys.exit()
                    else:
                        return selection  # Return selected map layout
            
            self.draw()
            self.clock.tick(60)
        
        return 0  # Default to first layout
